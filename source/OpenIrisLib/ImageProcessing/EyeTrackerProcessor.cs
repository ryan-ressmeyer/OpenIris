﻿//-----------------------------------------------------------------------
// <copyright file="EyeTrackerProcessor.cs">
//     Copyright (c) 2014-2023 Jorge Otero-Millan, Johns Hopkins University, University of California, Berkeley. All rights reserved.
// </copyright>
//-----------------------------------------------------------------------
namespace OpenIris
{
#nullable enable

    using System;
    using System.Collections.Concurrent;
    using System.Collections.Generic;
    using System.Threading;
    using System.Threading.Tasks;

    /// <summary>
    /// Class in charge of the background threads that processes images. It implements a multiplexed
    /// producer consumer. The processor is at the same time a producer and a consumer of images. It
    /// puts then in a concurrent queue and then several processing threads process the images and
    /// place them in another concurrent queue. Then, the output waiting list will go through 
    /// reorder the frames properly (they may have been finished processed out of order) and
    /// propagates the new data so other entities can use it.
    /// </summary>
    /// <remarks>
    /// The processor objects receives the frames from the <see cref="TryProcessImages"/> method and
    /// outputs the processed frames with the <see cref="ImagesProcessed"/> event.
    /// </remarks>
    public sealed class EyeTrackerProcessor
    {
        /// <summary>
        /// Possile modes of the processor.
        /// </summary>
        public enum Mode
        {
            /// <summary>
            /// Process in real time. May need to drop frames.
            /// </summary>
            RealTime,

            /// <summary>
            /// Process offline. Never drop frames.
            /// </summary>
            Offline,
        }

        private readonly Mode mode;
        private readonly int numberOfThreads;
        private readonly int inputBufferSize;
        private BlockingCollection<(EyeTrackerImagesAndData imagesAndData, long orderNumber)>? inputBuffer;
        private ConcurrentDictionary<long, EyeTrackerImagesAndData>? outputWaitingList;
        private int outputNextExpectedNumber = 0;
        private bool started;
        private bool stopping;

        /// <summary>
        /// Initializes an instance of the eyeTrackerProcessor class.
        /// </summary>
        /// <param name="processingMode">
        /// Value indicating weather frames can be dropped. This means the call to Process images
        /// will be blocking if the buffer is fulll.
        /// </param>
        /// <param name="bufferSize">Number of frames held in the buffer.</param>
        /// <param name="maxNumberOfThreads">Maximum number of threads to run.</param>
        public EyeTrackerProcessor(Mode processingMode, int maxNumberOfThreads = 1, int bufferSize = 1)
        {
            inputBufferSize = bufferSize;
            mode = processingMode;

            numberOfThreads = Math.Min(maxNumberOfThreads, Math.Max(1, (int)Math.Round(Environment.ProcessorCount / 2.0 - 1)));

            PipelineUI = new EyeCollection<EyeTrackingPipelineUIControl?>(null, null);
        }

        /// <summary>
        /// Notifies listeners that a frame has been processed and new data is available.
        /// </summary>
        internal event EventHandler<EyeTrackerImagesAndData>? ImagesProcessed;

        /// <summary>
        /// User interface for the current pipeline. For each eye.
        /// </summary>
        public EyeCollection<EyeTrackingPipelineUIControl?> PipelineUI { get; private set; }

        /// <summary>
        /// Gets the total number of frames received.
        /// </summary>
        public int NumberFramesReceived { get; private set; }

        /// <summary>
        /// Gets the total number of frames processed.
        /// </summary>
        public int NumberFramesProcessed { get; private set; }

        /// <summary>
        /// Gets the total number of frames dropped.
        /// </summary>
        public int NumberFramesNotProcessed { get => NumberFramesReceived - NumberFramesProcessed; }

        /// <summary>
        /// Gets the total number of frames  currently in the buffer.
        /// </summary>
        public int NumberFramesInBuffer => inputBuffer?.Count + outputWaitingList?.Count ?? 0;

        /// <summary>
        /// Gets a string with a status message regarding the processing.
        /// </summary>
        public string ProcessingStatus => $"Tracking " +
            $"[Frames in buffer:{NumberFramesInBuffer}, " +
            $"Dropped:{NumberFramesNotProcessed} " +
            $"({Math.Round(100.0 * NumberFramesNotProcessed / (double)(NumberFramesProcessed + NumberFramesNotProcessed))}%)]";

        /// <summary>
        /// Starts the processing threads.
        /// </summary>
        internal async Task Start()
        {
            if (started) throw new InvalidOperationException("Cannot start the processor again. Already running.");
            started = true;

            var processingTasks = new List<Task>();
            var errorHandler = new TaskErrorHandler(Stop);

            try
            {
                // Initialize buffers and threads. One to process the output queue and several to process the
                // images.
                using (inputBuffer = new BlockingCollection<(EyeTrackerImagesAndData, long)>(inputBufferSize))
                {
                    outputWaitingList = new ConcurrentDictionary<long, EyeTrackerImagesAndData>();

                    for (int i = 0; i < numberOfThreads; i++)
                    {
                        // Not using a using statement here because the ProcessInputLoop will dispose
                        // of this events. Otherwise they get disposed before the WhenAll
                        var newImagesLeftEvent = new AutoResetEvent(false);
                        var newImagesRightEvent = new AutoResetEvent(false);
                        var leftEyeDoneEvent = new AutoResetEvent(false);
                        var rightEyeDoneEvent = new AutoResetEvent(false);

                        EyeTrackerImagesAndData? currentImagesAndData = null;
                        IEyeTrackingPipeline pipelineLeft = new EyeTrackingPipelineNothing();
                        IEyeTrackingPipeline pipelineRight = new EyeTrackingPipelineNothing();

                        processingTasks.Add(Task.Factory.StartNew(() => ProcessOneEyeLoop(ref pipelineLeft, ref currentImagesAndData, Eye.Left, newImagesLeftEvent, leftEyeDoneEvent),
                            TaskCreationOptions.LongRunning)
                            .ContinueWith(errorHandler.HandleError));

                        processingTasks.Add(Task.Factory.StartNew(() => ProcessOneEyeLoop(ref pipelineRight, ref currentImagesAndData, Eye.Right, newImagesRightEvent, rightEyeDoneEvent),
                            TaskCreationOptions.LongRunning)
                            .ContinueWith(errorHandler.HandleError));

                        processingTasks.Add(Task.Factory.StartNew(()=>ProcessLoop(ref pipelineLeft, ref pipelineRight, ref currentImagesAndData, newImagesLeftEvent, leftEyeDoneEvent, newImagesRightEvent, rightEyeDoneEvent),
                            TaskCreationOptions.LongRunning)
                            .ContinueWith(errorHandler.HandleError));
                    }

                    await Task.WhenAll(processingTasks);

                    errorHandler.CheckForErrors();
                }
            }
            finally
            {
                processingTasks?.ForEach(t => t?.Dispose());
                inputBuffer = null;
                outputWaitingList = null;
            }
        }

        /// <summary>
        /// Stops the processing.
        /// </summary>
        public void Stop()
        {
            // Do not add more items to the input queue. Marking the queue with complete adding will
            // cause that the processingThreads threads to finish when the input queue is empty.
            inputBuffer?.CompleteAdding();
        }

        /// <summary>
        /// Adds a frame to the processing queue, if it is full waits or not dependening if the
        /// processing is configured to allow dropped frames.
        /// </summary>
        /// <param name="imagesAndData">Images to be processed.</param>
        /// <returns>
        /// True if the images were queued for processing. False if the frames were dropped because
        /// the buffere was full
        /// </returns>
        internal bool TryProcessImages(EyeTrackerImagesAndData imagesAndData)
        {
            if (inputBuffer is null) throw new InvalidOperationException("Buffer not ready.");

            NumberFramesReceived++;

            if (inputBuffer.IsAddingCompleted) return false;

            // Add the images to the input queue. The option allowDroppedFrames is used for realTime
            // processing. The thread will get not blocked here if there is no room in the
            // buffer.

            var result = true;

            switch (mode)
            {
                case Mode.RealTime:
                    result = inputBuffer.TryAdd((imagesAndData, NumberFramesProcessed));
                    break;
                case Mode.Offline:
                    inputBuffer.Add((imagesAndData, NumberFramesProcessed));
                    break;
            }

            if (result) NumberFramesProcessed++;

            return result;
        }

        /// <summary>
        /// Process loop that runs on a separate thread processing each frame whenever available in
        /// the buffer. Each image of left and right eye are processed themselves in different
        /// threads (Tasks).
        /// </summary>
        private void ProcessLoop(ref IEyeTrackingPipeline pipelineLeft, ref IEyeTrackingPipeline pipelineRight, ref EyeTrackerImagesAndData? currentImagesAndData, 
            AutoResetEvent newImagesLeftEvent, AutoResetEvent leftEyeDoneEvent, AutoResetEvent newImagesRightEvent, AutoResetEvent rightEyeDoneEvent)
        {
            Thread.CurrentThread.Name = "EyeTracker:ProcessLoop";

            if (inputBuffer is null) throw new InvalidOperationException("Buffer not ready.");
            if (outputWaitingList is null) throw new InvalidOperationException("Buffer not ready.");

            try
            {
                // Keep processing images until the buffer is marked as complete and empty
                using var cancellation = new CancellationTokenSource();
                foreach (var (imagesAndData, orderNumber) in inputBuffer.GetConsumingEnumerable(cancellation.Token))
                {
                    // 
                    // Check if it is necessary to change the pipeline and the corresponding UI
                    //
                    var currentPipelineName = imagesAndData.TrackingSettings.EyeTrackingPipelineName;

                    if (pipelineLeft?.Name != currentPipelineName | pipelineRight?.Name != currentPipelineName)
                    {
                        pipelineLeft = EyeTrackerPluginManager.EyeTrackingPipelineFactory?.Create(currentPipelineName) ?? new EyeTrackingPipelineNothing();
                        pipelineRight = EyeTrackerPluginManager.EyeTrackingPipelineFactory?.Create(currentPipelineName) ?? new EyeTrackingPipelineNothing();

                        // need this condition also because there may be many threads
                        // only one needs to change the UI but all of them need to change
                        // the pipeline
                        if (PipelineUI[Eye.Left]?.PipelineName != pipelineLeft?.Name | PipelineUI[Eye.Right]?.PipelineName != pipelineRight?.Name)
                        {
                            PipelineUI = new EyeCollection<EyeTrackingPipelineUIControl?>(
                                pipelineLeft?.GetPipelineUI(Eye.Left, pipelineLeft.Name), 
                                pipelineRight?.GetPipelineUI(Eye.Right, pipelineRight.Name));
                        }
                    }

                    //
                    // Wait for left and right eye to process
                    //
                    // save the reference to the current images for the left and right processing loops.
                    // use the events to let the left and right processes know they can go and work on 
                    // the image
                    currentImagesAndData = imagesAndData;
                    newImagesLeftEvent.Set();
                    newImagesRightEvent.Set();

                    // Wait for the left eye and right eye
                    leftEyeDoneEvent.WaitOne(); 
                    rightEyeDoneEvent.WaitOne(); 

                    //
                    // Propagate the processed images
                    //
                    if (numberOfThreads == 1 | orderNumber == outputNextExpectedNumber)
                    {
                        // if only one thread no need to use the output queue
                        // because the frames are not going to be out of order
                        // Same is this is the next frame we were expecting.
                        ImagesProcessed?.Invoke(this, currentImagesAndData);
                        outputNextExpectedNumber++;
                    }
                    else
                    {
                        // Add the processed images and send to the output queue for reordering
                        outputWaitingList.TryAdd(orderNumber, currentImagesAndData);

                        // Go thru the waiting list to look for next expected order number. If the image we
                        // are waiting for is in the waiting list. Remove the item and raise an event
                        // notifying that a new image was processed
                        while (outputWaitingList.TryRemove(outputNextExpectedNumber, out EyeTrackerImagesAndData images))
                        {
                            ImagesProcessed?.Invoke(this, images);
                            outputNextExpectedNumber++;
                        }
                    }
                }
                System.Diagnostics.Trace.WriteLine($"Processing loop finished.");
            }
            finally
            {
                stopping = true;

                currentImagesAndData = null;

                // Let the left and right tasks go so they can finish.
                newImagesLeftEvent.Set();
                newImagesRightEvent.Set();

                // Wait for the two threads to signal they completed their loops
                // before we dispose the events
                leftEyeDoneEvent.WaitOne();
                rightEyeDoneEvent.WaitOne();

                newImagesLeftEvent.Dispose();
                newImagesRightEvent.Dispose();
                leftEyeDoneEvent.Dispose();
                rightEyeDoneEvent.Dispose();

                inputBuffer = null;
            }
        }

        private void ProcessOneEyeLoop(ref IEyeTrackingPipeline pipeline, ref EyeTrackerImagesAndData? imagesAndData, Eye whichEye, AutoResetEvent newImageEvent, AutoResetEvent doneWithProcessingEvent)
        {
            Thread.CurrentThread.Name = "EyeTracker:EyeProcessLoop";

            while (stopping is false)
            {
                try
                {
                    // wait for new frame
                    newImageEvent.WaitOne();

                    var image = imagesAndData?.Images[whichEye];

                    if (image is null) continue;

                    EyeTrackerDebug.TrackTimeBeginPipeline(whichEye, image.TimeStamp);
                    (image.EyeData, image.ImageTorsion) = pipeline.Process(image, imagesAndData.Calibration.EyeCalibrationParameters[whichEye], imagesAndData.TrackingSettings);
                    EyeTrackerDebug.TrackTimeEndPipeline();
                }
                finally
                {
                    // signal that we are done with processing
                    doneWithProcessingEvent.Set();
                }
            }
            System.Diagnostics.Trace.WriteLine($"Thread for {whichEye} eye finished.");
        }
    }
}