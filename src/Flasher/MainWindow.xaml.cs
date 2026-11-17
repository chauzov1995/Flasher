using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Text.Json;
using System.Windows.Controls.Primitives;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;

namespace WpfApp1;

public partial class MainWindow : Window
{
    private const int BoardCount = 5;
    private readonly List<FlashInfo> rows;
    private readonly Properties.Settings settings = Properties.Settings.Default;
    private bool flashing;

    private readonly string root;
    private readonly string dataDirectory;

    public MainWindow() : this(AppContext.BaseDirectory, LocalPreset.DataDirectory) { }

    internal MainWindow(string root, string dataDirectory)
    {
        this.root = root;
        this.dataDirectory = dataDirectory;
        InitializeComponent();
        try
        {
            rows = JsonSerializer.Deserialize<List<FlashInfo>>(settings.spifflash);
        }
        catch (JsonException) { }
        rows ??= new List<FlashInfo>();
        rows.RemoveAll(row => row == null);
        while (rows.Count < BoardCount) rows.Add(new FlashInfo());
        foreach (var row in rows) row.RestoreLog(Path.Combine(dataDirectory, "logs"));
        chk_erase.IsChecked = settings.erase;
        chk_erase_Copy.IsChecked = settings.monitoe;
        chk_noClose.IsChecked = settings.noClosing;
        RefreshCatalog();
    }

    private void RefreshCatalog()
    {
        var profiles = new List<FirmwareProfile>();
        var messages = new List<string>();
        foreach (var folder in new[] { "projects", "projectsard" })
        {
            var path = Path.Combine(root, folder);
            if (!Directory.Exists(path))
            {
                if (folder == "projects") messages.Add($"Отсутствует каталог: {path}");
                continue;
            }
            foreach (var dir in Directory.GetDirectories(path).OrderBy(p => p))
            {
                if (folder == "projectsard" && !File.Exists(Path.Combine(dir, "ino.json"))) continue;
                try
                {
                    var profile = FirmwareProfile.Load(dir);
                    if (profiles.Any(p => p.NameDir.Equals(profile.NameDir, StringComparison.OrdinalIgnoreCase)))
                        throw new InvalidDataException($"Повторяющееся имя: {profile.NameDir}");
                    try { using var verified = profile.Prepare(root, "", false); }
                    catch (Exception ex)
                    {
                        profile.ValidationError = ex.Message;
                        messages.Add($"{profile.NameDir}: {ex.Message}");
                    }
                    profiles.Add(profile);
                }
                catch (Exception e) { messages.Add($"{Path.GetFileName(dir)}: {e.Message}"); }
            }
        }

        string[] ports;
        try { ports = SerialPort.GetPortNames().OrderBy(p => p.Length).ThenBy(p => p).ToArray(); }
        catch (Exception ex) { ports = Array.Empty<string>(); messages.Add(ex.Message); }
        if (ports.Length == 0) messages.Add("COM-порты не обнаружены. Подключите плату и нажмите «Обновить».");
        portSummary.Text = $"COM-порты: {ports.Length}";
        portIndicator.Fill = ports.Length > 0 ? (Brush)FindResource("Accent") : Brushes.DarkGray;
        profileSummary.Text = $"Профили: {profiles.Count(p => p.ValidationError == null)}";
        foreach (var message in messages) AppendLog(message);
        lb.ItemsSource = null;
        for (var i = 0; i < rows.Count; i++)
        {
            var row = rows[i];
            row.PropertyChanged -= Row_Changed;
            row.elem = i + 1;
            row.DirectoryProject = profiles;
            row.portNames = ports;
            if (!row.Connected || (row.Path != null && row.SelectedProfile == null) || row.SelectedProfile?.ValidationError != null)
            {
                StopMonitor(row);
                if (row.Stage == FlashStage.Success)
                    row.SetStage(FlashStage.Historical, row.Connected ? "Комплект недоступен" : "После записи порт отключался");
            }
            if (!profiles.Any(p => p.NameDir == row.Path)) row.Path = null;
            row.CatalogChanged();
            row.PropertyChanged += Row_Changed;
        }
        lb.ItemsSource = rows;
        UpdateSelection();
    }

    private void Row_Changed(object sender, PropertyChangedEventArgs e)
    {
        if (e.PropertyName is nameof(FlashInfo.Path) or nameof(FlashInfo.Port)) StopMonitor((FlashInfo)sender);
        if (e.PropertyName == nameof(FlashInfo.chk) || e.PropertyName == nameof(FlashInfo.Ready))
            UpdateSelection();
    }

    private void UpdateSelection()
    {
        var selected = rows.Where(r => r.chk).ToArray();
        var ready = rows.Count(r => r.Ready);
        chk_all.IsEnabled = ready > 0 || selected.Length > 0;
        chk_all.IsChecked = selected.Length == 0 ? false : selected.Length == ready && selected.All(r => r.Ready) ? true : null;
        selectionSummary.Text = $"Выбрано: {selected.Length} из {rows.Count}";
        var incomplete = selected.FirstOrDefault(r => !r.Ready);
        var duplicate = selected.Where(r => r.Ready).GroupBy(r => r.Port, StringComparer.OrdinalIgnoreCase).FirstOrDefault(g => g.Count() > 1);
        string hint = "";
        if (incomplete != null) hint = $"Строка {incomplete.elem:00}: {incomplete.DisplayStatus}.";
        else if (duplicate != null) hint = $"Порт {duplicate.Key} выбран несколько раз.";
        selectionHint.Text = hint;
        selectionHint.Visibility = hint.Length == 0 ? Visibility.Collapsed : Visibility.Visible;
        flash_all.IsEnabled = selected.Length > 0 && hint.Length == 0;
    }

    private void Workspace_SizeChanged(object sender, SizeChangedEventArgs e)
    {
        bool compact = e.NewSize.Width < 940;
        railColumn.Width = new GridLength(compact ? 0 : 318);
        Grid.SetColumn(rail, compact ? 0 : 1);
        Grid.SetRow(rail, compact ? 1 : 0);
        rail.Margin = compact ? new Thickness(0, 14, 0, 0) : new Thickness(16, 0, 0, 0);
    }

    private void RowMenu_Click(object sender, RoutedEventArgs e)
    {
        var button = (Button)sender;
        button.ContextMenu.PlacementTarget = button;
        button.ContextMenu.Placement = PlacementMode.Bottom;
        button.ContextMenu.IsOpen = true;
    }

    private void Update_Click(object sender, RoutedEventArgs e)
    {
        RefreshCatalog();
    }

    private void Clear_Click(object sender, RoutedEventArgs e)
    {
        if (MessageBox.Show("Очистить выбор прошивок и COM-портов?", Title, MessageBoxButton.YesNo) != MessageBoxResult.Yes)
            return;
        StopAllMonitors();
        txt_log.Clear();
        rows.Clear();
        for (int i = 0; i < BoardCount; i++) rows.Add(new FlashInfo());
        chk_all.IsChecked = false;
        RefreshCatalog();
    }

    private void chk_all_Click(object sender, RoutedEventArgs e)
    {
        bool select = rows.Any(row => row.Ready && !row.chk) || rows.Any(row => !row.Ready && row.chk);
        foreach (var row in rows) row.chk = select && row.Ready;
        UpdateSelection();
    }

    private async void cmdFlash_Clicked(object sender, RoutedEventArgs e) =>
        await FlashAsync(new[] { (FlashInfo)((FrameworkElement)sender).DataContext });

    private async void flash_all_Click(object sender, RoutedEventArgs e) =>
        await FlashAsync(rows.Where(row => row.chk).ToArray());

    private async Task FlashAsync(FlashInfo[] selected)
    {
        if (flashing) return;
        flashing = true;
        queuePanel.IsEnabled = writeOptions.IsEnabled = Update.IsEnabled = false;
        var jobs = new List<(FlashInfo Row, FlashPackage Package)>();
        try
        {
            var ports = SerialPort.GetPortNames();
            if (selected.Length == 0) throw new InvalidOperationException("Отметьте строки для прошивки.");
            foreach (var row in selected)
            {
                var profile = row.SelectedProfile;
                if (profile == null || !ports.Contains(row.Port, StringComparer.OrdinalIgnoreCase))
                    throw new InvalidOperationException($"Строка {row.elem}: выберите прошивку и подключённый COM-порт.");
                if (jobs.Any(j => string.Equals(j.Row.Port, row.Port, StringComparison.OrdinalIgnoreCase)))
                    throw new InvalidOperationException($"Порт {row.Port} выбран несколько раз.");
                try { jobs.Add((row, profile.Prepare(root, row.Port, chk_erase.IsChecked == true))); }
                catch (Exception ex)
                {
                    throw new InvalidDataException($"Строка {row.elem}, {row.Path}: {ex.Message}", ex);
                }
            }
            foreach (var (row, package) in jobs) await FlashRow(row, package.Command);
        }
        catch (Exception ex)
        {
            logSource.Text = "Запись не начата";
            AppendLog(ex.Message);
            RefreshCatalog();
        }
        finally
        {
            foreach (var job in jobs) job.Package.Dispose();
            flashing = false;
            queuePanel.IsEnabled = writeOptions.IsEnabled = Update.IsEnabled = true;
        }
    }

    private async Task FlashRow(FlashInfo row, ProcessStartInfo command)
    {
        StopPort(row.Port);
        foreach (var previous in rows.Where(other => other != row && string.Equals(other.Port, row.Port, StringComparison.OrdinalIgnoreCase) && other.Stage == FlashStage.Success))
            previous.SetStage(FlashStage.Historical, $"Порт использован в строке {row.elem:00}");
        row.flMAC = null;
        row.SetLog(null);
        row.SetStage(FlashStage.Writing);
        logSource.Text = $"Запись · {row.Path} · {row.Port}";
        try
        {
            string directory = Path.Combine(dataDirectory, "logs");
            Directory.CreateDirectory(directory);
            var path = Path.Combine(directory, $"{DateTime.Now:yyyyMMdd-HHmmssfff}-{row.Port}-{row.Path}-{Guid.NewGuid():N}.txt");
            using (var log = File.CreateText(path))
            {
                row.SetLog(path);
                log.AutoFlush = true;
                var header = $"{DateTimeOffset.Now:O} · Строка {row.elem:00} · {row.Path} · {row.Port}";
                log.WriteLine(header);
                AppendLog(header);
                try
                {
                    var exitCode = await RunTool(row, command, log);
                    string result = exitCode == 0 ? "Прошито" : $"Ошибка esptool ({exitCode})";
                    log.WriteLine($"{DateTimeOffset.Now:O} · {result} · MAC {row.flMAC ?? "не получен"}");
                    row.SetStage(exitCode == 0 ? FlashStage.Success : FlashStage.Failure, result);
                }
                catch (Exception ex) { log.WriteLine("Ошибка: " + ex.Message); throw; }
            }
        }
        catch (Exception ex) { row.SetStage(FlashStage.Failure, "Ошибка: " + ex.Message); }
        logSource.Text = $"{row.Port} · {row.DisplayStatus}";
        AppendLog(logSource.Text);
        if (row.Stage == FlashStage.Success && chk_erase_Copy.IsChecked == true) OpenMonitor(row);
        if (row.HasLog && chk_noClose.IsChecked == true)
        {
            try { OpenLog(row); }
            catch (Exception ex) { AppendLog("Журнал: " + ex.Message); }
        }
    }

    private async Task<int> RunTool(FlashInfo row, ProcessStartInfo command, StreamWriter log)
    {
        using var process = Process.Start(command) ?? throw new IOException("Не удалось запустить esptool.");
        IOException logError = null;
        async Task ReadOutput(StreamReader stream)
        {
            while (await stream.ReadLineAsync() is string line)
            {
                if (logError == null)
                {
                    try { log.WriteLine(line); }
                    catch (IOException ex) { logError = ex; }
                }
                AppendLog($"[{row.Port}] {line}");
                int macPrefix = line.StartsWith("BASE MAC: ", StringComparison.OrdinalIgnoreCase) ? 10 :
                    line.StartsWith("MAC: ", StringComparison.OrdinalIgnoreCase) ? 5 : 0;
                if (macPrefix != 0)
                {
                    var mac = line[macPrefix..].Trim().Replace(":", "").ToUpperInvariant();
                    if (LocalPreset.ValidMac(mac)) row.flMAC = mac;
                }
            }
        }
        await Task.WhenAll(ReadOutput(process.StandardOutput), ReadOutput(process.StandardError), process.WaitForExitAsync());
        if (logError != null) throw logError;
        return process.ExitCode;
    }

    private void AppendLog(string line)
    {
        const int limit = 96000;
        if (txt_log.Text.Length + line.Length >= limit) txt_log.Text = txt_log.Text[Math.Min(txt_log.Text.Length, limit / 2)..];
        if (line.Length > limit / 2) line = line[^(limit / 2)..];
        txt_log.AppendText(line + Environment.NewLine);
        txt_log.ScrollToEnd();
    }

    private static void OpenLog(FlashInfo row) =>
        Process.Start(new ProcessStartInfo(row.LastLog) { UseShellExecute = true });

    private void cmdLog_Clicked(object sender, RoutedEventArgs e)
    {
        var row = (FlashInfo)((FrameworkElement)sender).DataContext;
        try { OpenLog(row); }
        catch (Exception ex) { MessageBox.Show(this, ex.Message, Title); }
    }

    private void cmd_Settings_Clicked(object sender, RoutedEventArgs e)
    {
        var row = (FlashInfo)((FrameworkElement)sender).DataContext;
        if (!row.CanEditPreset) return;
        try
        {
            new Settings(row.Path, row.flMAC, Path.Combine(dataDirectory, "presets")) { Owner = this }.ShowDialog();
        }
        catch (Exception ex) { MessageBox.Show(this, ex.Message, Title); }
    }

    protected override void OnClosing(CancelEventArgs e)
    {
        base.OnClosing(e);
        if (!flashing) return;
        e.Cancel = true;
        MessageBox.Show("Дождитесь завершения прошивки.", Title);
    }

    private void Window_Closed(object sender, EventArgs e)
    {
        StopAllMonitors();
        settings.spifflash = JsonSerializer.Serialize(rows);
        settings.erase = chk_erase.IsChecked == true;
        settings.noClosing = chk_noClose.IsChecked == true;
        settings.monitoe = chk_erase_Copy.IsChecked == true;
        settings.Save();
    }
}
