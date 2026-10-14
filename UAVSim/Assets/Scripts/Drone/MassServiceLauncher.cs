using System;
using System.Diagnostics;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;
using Debug = UnityEngine.Debug;

/// <summary>
/// Starts the mass-properties service (udp_listener.py) from the Python bundled under
/// MassService~ / StreamingAssets/MassService, so the student never runs it by hand.
///
/// Plain static class, creates no GameObjects. CustomFrameLoader and the wire protocol are
/// untouched: this only makes sure something is answering on 127.0.0.1:5070 before Begin.
///
/// EnsureRunning() is idempotent and returns at once; the probe/spawn/wait runs on a worker
/// thread. Callers read Current from the main thread (ImportPageUI polls it in Update).
/// </summary>
public static class MassServiceLauncher
{
    public enum State
    {
        NotStarted,
        Starting,
        Ready,
        Failed
    }

    public struct Status
    {
        public State state;
        public string message;
        /// <summary>Incremented on every change, so pollers can cheaply detect one.</summary>
        public int version;
    }

    private const int Port = 5070;
    private const byte ProbeByte = 0;          // anything but 40 gets a 255 error reply
    private const byte ErrorHeader = 255;
    private const int ProbeTimeoutMilliseconds = 300;
    private const int ReadyTimeoutMilliseconds = 10000;
    private const string LogFileName = "mass-service.log";

    private static readonly object Gate = new object();
    private static Status status;
    private static bool workerRunning;

    // Only ever a process this class started. Shutdown never touches one it did not start
    // (e.g. a service the student launched by hand).
    private static Process ownedProcess;
    private static TextWriter logWriter;

    public static Status Current
    {
        get { lock (Gate) return status; }
    }

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
    private static void Initialize()
    {
        Application.quitting -= Shutdown;
        Application.quitting += Shutdown;
    }

    /// <summary>
    /// Makes sure the service is up, starting the bundled one if nothing answers. Safe to call
    /// repeatedly (ImportPageUI.OnEnable fires on every Back round trip): a check already in
    /// progress is left alone, and a Ready or Failed state is re-checked.
    /// </summary>
    public static void EnsureRunning()
    {
        lock (Gate)
        {
            if (workerRunning)
                return;
            workerRunning = true;
        }

        // Unity paths must be read on the main thread.
        string serviceRoot = ResolveServiceRoot();
        string logPath = Path.Combine(Application.persistentDataPath, LogFileName);
        bool isWindows = Application.platform == RuntimePlatform.WindowsPlayer ||
                         Application.platform == RuntimePlatform.WindowsEditor;

        var worker = new Thread(() => Run(serviceRoot, logPath, isWindows))
        {
            IsBackground = true,
            Name = "MassServiceLauncher"
        };
        worker.Start();
    }

    /// <summary>
    /// Folder holding python/ and mass-properties/. In the Editor it is the importer-ignored
    /// Assets/MassService~; in a player MassServiceBuildStep copies it to StreamingAssets.
    /// </summary>
    public static string ResolveServiceRoot()
    {
#if UNITY_EDITOR
        return Path.Combine(Application.dataPath, "MassService~");
#else
        return Path.Combine(Application.streamingAssetsPath, "MassService");
#endif
    }

    private static void Run(string serviceRoot, string logPath, bool isWindows)
    {
        try
        {
            if (Probe())
            {
                SetStatus(State.Ready, "ready");
                return;
            }

            bool ownedAlive;
            lock (Gate) ownedAlive = ownedProcess != null && !ownedProcess.HasExited;

            if (!ownedAlive)
            {
                if (!isWindows)
                {
                    Fail("the bundled service is Windows only - start udp_listener.py by hand");
                    return;
                }

                string pythonExe = Path.Combine(serviceRoot, "python", "python.exe");
                string scriptDir = Path.Combine(serviceRoot, "mass-properties");
                string script = Path.Combine(scriptDir, "udp_listener.py");
                if (!File.Exists(pythonExe) || !File.Exists(script))
                {
                    Fail($"bundled Python not found under {serviceRoot} - run tools/build-mass-service.ps1");
                    return;
                }

                SetStatus(State.Starting, "starting...");
                Spawn(pythonExe, script, scriptDir, logPath);
            }
            else
            {
                SetStatus(State.Starting, "starting...");
            }

            WaitUntilReady(logPath);
        }
        catch (Exception e)
        {
            Fail(e.Message);
        }
        finally
        {
            lock (Gate) workerRunning = false;
        }
    }

    private static void Spawn(string pythonExe, string script, string scriptDir, string logPath)
    {
        var startInfo = new ProcessStartInfo
        {
            FileName = pythonExe,
            // -u: unbuffered, so the log is written as it happens rather than lost on a kill.
            // --parent-pid: the service exits by itself if Unity dies without calling Shutdown.
            Arguments = $"-u \"{script}\" --parent-pid {Process.GetCurrentProcess().Id}",
            WorkingDirectory = scriptDir,
            UseShellExecute = false,
            CreateNoWindow = true,
            RedirectStandardOutput = true,
            RedirectStandardError = true
        };

        TextWriter writer = TextWriter.Synchronized(new StreamWriter(logPath, false) { AutoFlush = true });
        writer.WriteLine($"[{DateTime.Now:yyyy-MM-dd HH:mm:ss}] {startInfo.FileName} {startInfo.Arguments}");

        var process = new Process { StartInfo = startInfo };
        process.OutputDataReceived += (_, e) => WriteLog(writer, e.Data);
        process.ErrorDataReceived += (_, e) => WriteLog(writer, e.Data);
        process.Start();
        process.BeginOutputReadLine();
        process.BeginErrorReadLine();

        lock (Gate)
        {
            ownedProcess = process;
            logWriter = writer;
        }
        Debug.Log($"[MassServiceLauncher] Started bundled mass service (pid {process.Id}). Log: {logPath}");
    }

    private static void WriteLog(TextWriter writer, string line)
    {
        if (line == null)
            return;
        try
        {
            writer.WriteLine(line);
        }
        catch (ObjectDisposedException)
        {
            // Output still draining after Shutdown closed the log; nothing left to keep.
        }
    }

    private static void WaitUntilReady(string logPath)
    {
        var timer = Stopwatch.StartNew();
        while (timer.ElapsedMilliseconds < ReadyTimeoutMilliseconds)
        {
            if (Probe())
            {
                SetStatus(State.Ready, "ready");
                Debug.Log($"[MassServiceLauncher] Mass service ready after {timer.ElapsedMilliseconds} ms.");
                return;
            }

            Process process;
            lock (Gate) process = ownedProcess;
            if (process != null && process.HasExited)
            {
                Fail($"service exited with code {process.ExitCode} - see {logPath}");
                return;
            }

            // Probe() already waits up to ProbeTimeoutMilliseconds when the port is open but
            // silent; a closed port fails at once, so pace the loop here too.
            Thread.Sleep(100);
        }

        Fail($"no reply on 127.0.0.1:{Port} after {ReadyTimeoutMilliseconds / 1000} s - see {logPath}");
    }

    /// <summary>
    /// True if a mass service answers on 127.0.0.1:5070. Sends one byte that is not the request
    /// header; the service replies to any other header with a 255 error packet, so this never
    /// touches the locked request format. A closed port shows up as a connection reset.
    /// </summary>
    private static bool Probe()
    {
        try
        {
            using (var client = new UdpClient())
            {
                client.Client.ReceiveTimeout = ProbeTimeoutMilliseconds;
                var endpoint = new IPEndPoint(IPAddress.Loopback, Port);
                client.Send(new[] { ProbeByte }, 1, endpoint);
                var remote = new IPEndPoint(IPAddress.Any, 0);
                byte[] reply = client.Receive(ref remote);
                return reply.Length > 0 && reply[0] == ErrorHeader;
            }
        }
        catch (SocketException)
        {
            return false;
        }
    }

    private static void SetStatus(State state, string message)
    {
        lock (Gate)
        {
            if (status.state == state && status.message == message)
                return;
            status.state = state;
            status.message = message;
            status.version++;
        }
    }

    private static void Fail(string reason)
    {
        SetStatus(State.Failed, reason);
        Debug.LogError($"[MassServiceLauncher] Mass service failed to start: {reason}. " +
                       "Custom frames will fall back to the default drone.");
    }

    private static void Shutdown()
    {
        Process process;
        TextWriter writer;
        lock (Gate)
        {
            process = ownedProcess;
            writer = logWriter;
            ownedProcess = null;
            logWriter = null;
            status = default;
        }

        try
        {
            if (process != null && !process.HasExited)
            {
                process.Kill();
                process.WaitForExit(1000);
            }
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[MassServiceLauncher] Could not stop the mass service: {e.Message}");
        }
        finally
        {
            process?.Dispose();
            writer?.Dispose();
        }
    }
}
