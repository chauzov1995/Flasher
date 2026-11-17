using System;
using System.Collections.Generic;
using System.Linq;

namespace WpfApp1;

internal enum TestState { Waiting, Passed, Failed }
internal sealed class TestOutput(string profile)
{
    private static readonly string[] FanErrors =
    {
        "ccs co2 levie dannie", "ccs co2 net dannih", "ccs co2 ne viden na shine",
        "sht20 does not show hum", "sht20 does not show temp", "illuminat does not show", "no wifi"
    };
    private static readonly string[] Pending = { "btn_", "moter ", "ina_napr", "mintok", "maxtok", "temp_" };
    private readonly List<string> fanDetails = new();
    public TestState State { get; private set; }
    public string Text { get; private set; }

    public bool Accept(string raw)
    {
        var line = raw.Trim();
        if (profile == "testgreen2")
        {
            if (line.StartsWith("GREEN2_TEST REGISTERED id=GN_", StringComparison.Ordinal))
                return Set(TestState.Passed, "Зарегистрировано · моторы не проверялись");
            if (line.StartsWith("GREEN2_TEST ERROR ", StringComparison.Ordinal))
                return Set(TestState.Failed, "Ошибка: " + line[18..]);
            if (line.StartsWith("GREEN2_TEST READY ", StringComparison.Ordinal))
                return Set(TestState.Waiting, "Готово к регистрации · удержите SYNC 2 с");
            if (line.StartsWith("GREEN2_TEST WAIT ", StringComparison.Ordinal))
                return Set(TestState.Waiting, "Ожидание: " + line[17..]);
            if (line.StartsWith("GREEN2_TEST BOOT ", StringComparison.Ordinal))
                return Set(TestState.Waiting, "Тест Green2 запущен · регистрация ещё не проверена");
            return false;
        }
        if (profile == "testirovanieFan")
        {
            if (line == "succes")
            {
                fanDetails.Clear();
                return Set(TestState.Passed, "Успех");
            }
            if (line == "error")
            {
                var text = fanDetails.Count == 0 ? "Ошибка теста" : "Ошибка: " + string.Join(", ", fanDetails);
                fanDetails.Clear();
                return Set(TestState.Failed, text);
            }
            if (FanErrors.Any(prefix => line.StartsWith(prefix, StringComparison.Ordinal)))
            {
                if (fanDetails.Count < FanErrors.Length && !fanDetails.Contains(line)) fanDetails.Add(line);
                return Set(TestState.Failed, "Ошибка: " + string.Join(", ", fanDetails));
            }
        }
        if (profile is "testfenster" or "testteplica")
        {
            if (line == "OK") return Set(TestState.Passed, "Успех");
            if (line is "not connect" or "Unable to create client" or "could not connect. Fix and Reboot" ||
                line.StartsWith("Error: Could not read temperature data") || line.StartsWith("[HTTPS] GET... failed") ||
                line.StartsWith("HTTP_UPDATE_FAILED") ||
                (line.StartsWith("[HTTPS] GET... code: ") && line != "[HTTPS] GET... code: 200"))
                return Set(TestState.Failed, "Ошибка: " + line);
            if (Pending.Any(prefix => line.StartsWith(prefix, StringComparison.Ordinal)))
                return Set(TestState.Waiting, "Ожидание проверки: " + line);
        }
        return false;
    }

    private bool Set(TestState state, string text)
    {
        if (State == state && Text == text) return false;
        State = state;
        Text = text;
        return true;
    }
}
