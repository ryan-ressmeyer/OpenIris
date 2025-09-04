﻿
namespace OpenIris
{
    using System;
    using System.Net;
    using System.Net.Sockets;
    using System.Threading.Tasks;
    using System.Diagnostics;
    using System.Threading;

    internal class EyeTrackerTcpListener : IDisposable
    {
        private EyeTrackerRemote eyeTracker;
        private Int32 port;
        private TcpListener server = null;
        private Task task;
        private bool disposedValue;

        public EyeTrackerTcpListener(EyeTracker eyeTracker, int port)
        {
            this.port = port;
            this.eyeTracker = new EyeTrackerRemote(eyeTracker);
        }

        public void Start()
        {
            task = Task.Run(() =>
            {
                Thread.CurrentThread.Name = "EyeTracker:TCP server";
                Thread.CurrentThread.IsBackground = true; // Stops the thread from preventing the application from closing

                try
                {
                    // TcpListener server = new TcpListener(port);
                    server = new TcpListener(IPAddress.Any, port);

                    // Start listening for client requests.
                    server.Start();

                    // Buffer for reading data
                    Byte[] bytes = new Byte[1024];

                    // Enter the listening loop.
                    while (true)
                    {
                        Trace.WriteLine($"TCP server Waiting for a connection in port {port} ...");

                        // Perform a blocking call to accept requests.
                        // You could also use server.AcceptSocket() here.
                        using TcpClient client = server.AcceptTcpClient();
                        Trace.WriteLine("TCP server Connected!");

                        // Get a stream object for reading and writing
                        NetworkStream stream = client.GetStream();

                        int i;

                        // Loop to receive all the data sent by the client.
                        while ((i = stream.Read(bytes, 0, bytes.Length)) != 0)
                        {
                            // Create a new byte array containing only the received data.
                            byte[] receivedBytes = new byte[i];
                            Array.Copy(bytes, 0, receivedBytes, 0, i);

                            // Pass the correctly sized byte array to the parsing method.
                            var bytesToSend = eyeTracker.ParseAndExecuteStringMessage(receivedBytes);

                            if (bytesToSend.Length > 0)
                            {
                                // Send back a response.
                                stream.Write(bytesToSend, 0, bytesToSend.Length);
                            }
                        }
                    }
                }
                catch (SocketException e)
                {
                    Console.WriteLine("SocketException: {0}", e);
                }
                finally
                {
                    server.Stop();
                }
            });
        }

        public void Stop()
        {
            server.Stop();
            var tempTask = task;
            task = null;
            tempTask?.Wait();
            tempTask?.Dispose();
        }

        protected virtual void Dispose(bool disposing)
        {
            if (!disposedValue)
            {
                if (disposing)
                {
                    task.Dispose();
                }

                // TODO: free unmanaged resources (unmanaged objects) and override finalizer
                // TODO: set large fields to null
                disposedValue = true;
            }
        }

        // // TODO: override finalizer only if 'Dispose(bool disposing)' has code to free unmanaged resources
        // ~EyeTrackerTcpListener()
        // {
        //     // Do not change this code. Put cleanup code in 'Dispose(bool disposing)' method
        //     Dispose(disposing: false);
        // }

        public void Dispose()
        {
            // Do not change this code. Put cleanup code in 'Dispose(bool disposing)' method
            Dispose(disposing: true);
            GC.SuppressFinalize(this);
        }
    }
}
