using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Windows;
using System.Windows.Media;

namespace WpfApp1;

public partial class MainWindow
{
    private sealed class MonitorSession(FlashInfo row)
    {
        public readonly FlashInfo Row = row;
        public readonly string Port = row.Port;
        public readonly string Label = $"{row.Port} · {row.Path ?? "без профиля"}";
        public readonly SerialPort Serial = new(row.Port, 115200);
        public readonly TestOutput Result = new(row.SelectedProfile?.Kind == "test" ? row.Path : null);
        public string Tail = "";
        public SerialDataReceivedEventHandler Handler;
    }

    private readonly Dictionary<string, MonitorSession> monitors = new(StringComparer.OrdinalIgnoreCase);

    private void OpenMonitor(FlashInfo row)
    {
        if (monitors.TryGetValue(row.Port, out var active) && active.Row == row) return;
        StopPort(row.Port);
        MonitorSession session = null;
        try
        {
            session = new MonitorSession(row);
            session.Handler = (_, _) => Receive(session);
            monitors.Add(session.Port, session);
            session.Serial.DataReceived += session.Handler;
            session.Serial.Open();
            AppendLog($"[{session.Label}] Монитор открыт · 115200 бод");
            ShowMonitorStatus();
        }
        catch (Exception ex)
        {
            if (session != null) StopPort(session.Port);
            AppendLog($"[{row.Port}] Не удалось открыть монитор: {ex.Message}");
        }
    }

    private void Receive(MonitorSession session)
    {
        string data;
        try { data = session.Serial.ReadExisting(); }
        catch (Exception ex) when (ex is IOException or InvalidOperationException or UnauthorizedAccessException)
        {
            Dispatcher.InvokeAsync(() =>
            {
                if (!IsCurrent(session)) return;
                StopPort(session.Port);
                if (session.Row.Stage == FlashStage.Success)
                    session.Row.SetStage(FlashStage.Historical, "Связь после записи потеряна");
                AppendLog($"[{session.Port}] Ошибка чтения: {ex.Message}");
            });
            return;
        }

        Dispatcher.InvokeAsync(() =>
        {
            if (!IsCurrent(session)) return;
            var lines = (session.Tail + data).Split('\n');
            session.Tail = lines[^1];
            if (session.Tail.Length > 4096)
            {
                AppendLog($"[{session.Label}] {session.Tail[..^4096]}");
                session.Tail = session.Tail[^4096..];
            }
            bool changed = false;
            foreach (var raw in lines.SkipLast(1))
            {
                string line = raw.TrimEnd('\r');
                if (line.Length == 0) continue;
                AppendLog($"[{session.Label}] {line}");
                changed |= session.Result.Accept(line);
            }
            ShowPartialLines();
            if (changed) ShowResults();
        });
    }

    private bool IsCurrent(MonitorSession session) => monitors.TryGetValue(session.Port, out var active) && active == session;

    private void StopPort(string port)
    {
        if (port == null || !monitors.Remove(port, out var session)) return;
        session.Serial.DataReceived -= session.Handler;
        try { session.Serial.Dispose(); }
        catch (IOException ex) { AppendLog($"[{port}] {ex.Message}"); }
        if (session.Tail.Length > 0) AppendLog($"[{session.Label}] {session.Tail}");
        AppendLog($"[{session.Label}] Монитор остановлен");
        ShowMonitorStatus();
        ShowPartialLines();
        ShowResults();
    }

    private void ShowPartialLines()
    {
        var partials = monitors.Values.Where(s => s.Tail.Length > 0)
            .Select(s => $"[{s.Label}] {s.Tail.TrimEnd('\r')}");
        partialLog.Text = string.Join(Environment.NewLine, partials);
        partialLog.Visibility = partialLog.Text.Length == 0 ? Visibility.Collapsed : Visibility.Visible;
    }

    private void ShowMonitorStatus()
    {
        btn_Close_COM.Visibility = monitors.Count > 0 ? Visibility.Visible : Visibility.Collapsed;
        logSource.Text = monitors.Count > 0 ? "Монитор · " + string.Join(", ", monitors.Keys) : "Вывод esptool и COM-порта";
    }

    private void ShowResults()
    {
        var results = monitors.Values.Where(s => s.Result.Text != null).OrderBy(s => s.Row.elem).ToArray();
        txt_err.Text = string.Join(Environment.NewLine, results.Select(s => $"{s.Row.elem:00} · {s.Label} — {s.Result.Text}"));
        txt_err.Foreground = results.Any(s => s.Result.State == TestState.Failed) ? Brushes.Firebrick :
            results.Length > 0 && results.All(s => s.Result.State == TestState.Passed) ? Brushes.Green : (Brush)FindResource("Ink");
    }

    private void StopMonitor(FlashInfo row)
    {
        var session = monitors.Values.FirstOrDefault(s => s.Row == row);
        if (session != null) StopPort(session.Port);
    }

    private void StopAllMonitors()
    {
        foreach (var port in monitors.Keys.ToArray()) StopPort(port);
    }

    private void cmdMonitor_Clicked(object sender, RoutedEventArgs e)
    {
        var row = (FlashInfo)((FrameworkElement)sender).DataContext;
        if (row.Connected) OpenMonitor(row);
    }

    private void btn_Close_COM_Click(object sender, RoutedEventArgs e) => StopAllMonitors();
}
