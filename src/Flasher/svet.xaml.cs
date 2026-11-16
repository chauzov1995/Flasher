using System.Windows;

namespace WpfApp1;

public partial class Svet : Window
{
    private readonly DeviceIOT device;

    public Svet(DeviceIOT device)
    {
        InitializeComponent();
        this.device = device;
        txt_numPIN.Text = device.numPIN.ToString();
        txt_numBUTTON.Text = device.numButton.ToString();
        chk_reverseBULB.IsChecked = device.reverseBULB;
        chk_dimBRIGHTNESS.IsChecked = device.dimBRIGHTNESS;
        chk_SMOTH.IsChecked = device.SMOTH;
        chk_IKsenor.IsChecked = device.IKsenor;
    }

    private void btn_Save_Click(object sender, RoutedEventArgs e)
    {
        if (!int.TryParse(txt_numPIN.Text, out var numPIN) || !DeviceIOT.ValidGpio(numPIN, true) ||
            !int.TryParse(txt_numBUTTON.Text, out var numButton) || !DeviceIOT.ValidGpio(numButton, false))
        {
            MessageBox.Show(this, "Проверьте GPIO выходов и кнопок. GPIO 34–39 доступны только для входа.", Title);
            return;
        }
        device.numPIN = numPIN;
        device.numButton = numButton;
        device.reverseBULB = chk_reverseBULB.IsChecked == true;
        device.dimBRIGHTNESS = chk_dimBRIGHTNESS.IsChecked == true;
        device.SMOTH = chk_SMOTH.IsChecked == true;
        device.IKsenor = chk_IKsenor.IsChecked == true;
        Close();
    }
}
