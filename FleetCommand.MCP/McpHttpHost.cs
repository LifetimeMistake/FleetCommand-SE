using System;
using System.Diagnostics;
using System.IO;
using System.Net;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace FleetCommand.MCP
{
    public sealed class McpHttpHost : IDisposable
    {
        private readonly McpEndpoint _endpoint;
        private readonly HttpListener _listener;
        private readonly CancellationTokenSource _cts = new CancellationTokenSource();
        private readonly Action<string> _log;
        private Task _acceptLoop;
        private bool _started;

        public McpHttpHost(McpEndpoint endpoint, string prefix, Action<string> log = null)
        {
            _endpoint = endpoint ?? throw new ArgumentNullException(nameof(endpoint));
            Prefix = prefix ?? throw new ArgumentNullException(nameof(prefix));
            _log = log ?? (_ => { });
            _listener = new HttpListener();
            _listener.Prefixes.Add(prefix);
        }

        public string Prefix { get; }

        public void Start()
        {
            if (_started)
            {
                return;
            }
            _listener.Start();
            _started = true;
            _acceptLoop = Task.Run(() => AcceptLoopAsync());
        }

        private async Task AcceptLoopAsync()
        {
            while (!_cts.IsCancellationRequested)
            {
                HttpListenerContext context;
                try
                {
                    context = await _listener.GetContextAsync().ConfigureAwait(false);
                }
                catch (HttpListenerException)
                {
                    return;
                }

                _ = Task.Run(() => HandleAsync(context));
            }
        }

        private async Task HandleAsync(HttpListenerContext context)
        {
            var stopwatch = Stopwatch.StartNew();
            int status = 200;
            try
            {
                if (!string.Equals(context.Request.HttpMethod, "POST", StringComparison.OrdinalIgnoreCase))
                {
                    status = 405;
                }
                else
                {
                    string body;
                    using (var reader = new StreamReader(context.Request.InputStream, context.Request.ContentEncoding ?? Encoding.UTF8))
                    {
                        body = await reader.ReadToEndAsync().ConfigureAwait(false);
                    }

                    var result = await _endpoint.ProcessAsync(body, _cts.Token).ConfigureAwait(false);
                    if (result.WroteBody)
                    {
                        byte[] bytes = Encoding.UTF8.GetBytes(result.Body);
                        context.Response.ContentType = "text/event-stream";
                        context.Response.ContentLength64 = bytes.Length;
                        await context.Response.OutputStream.WriteAsync(bytes, 0, bytes.Length).ConfigureAwait(false);
                    }
                    else
                    {
                        status = 202;
                    }
                }
            }
            catch
            {
                status = 500;
            }
            finally
            {
                try { context.Response.StatusCode = status; } catch { }
                context.Response.Close();
                _log(string.Format("{0} {1} -> {2} in {3} ms",
                    context.Request.HttpMethod, context.Request.Url?.AbsolutePath ?? "/", status, stopwatch.ElapsedMilliseconds));
            }
        }

        public void Dispose()
        {
            if (_started)
            {
                _cts.Cancel();
                try { _listener.Stop(); } catch (HttpListenerException) { }
                _listener.Close();
            }
        }
    }
}
