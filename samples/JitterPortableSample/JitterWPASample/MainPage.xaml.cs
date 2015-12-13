using Windows.ApplicationModel.Activation;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Microsoft.Xna.Framework;
using MonoGame.Framework;

using JitterSample;
using Windows.Graphics.Display;

namespace JitterWPASample
{
    public sealed partial class MainPage : SwapChainBackgroundPanel
    {
        private readonly JitterPhysicsGame game;

        public MainPage()
        {
            InitializeComponent();
        }

        public MainPage(LaunchActivatedEventArgs e)
        {
            var ppv = DisplayInformation.GetForCurrentView().RawPixelsPerViewPixel;
            var bounds = Window.Current.CoreWindow.Bounds;
            JitterPhysicsGame.PreferredSize = new Vector2(
                (float)(bounds.Width * ppv),
                (float)(bounds.Height * ppv));

            game = XamlGame<JitterPhysicsGame>.Create(e, Window.Current.CoreWindow, this);
        }
    }
}
