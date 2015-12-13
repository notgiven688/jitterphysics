using Android.App;
using Android.Content.PM;
using Android.OS;
using Android.Views;
using Microsoft.Xna.Framework;

using JitterSample;

namespace JitterAndroidSample
{
    [Activity(
        Label = "@string/app_name",
        MainLauncher = true,
        Theme = "@style/Theme.Splash",
        AlwaysRetainTaskState = true,
        LaunchMode = LaunchMode.SingleInstance,
        ScreenOrientation = ScreenOrientation.SensorPortrait,
        ConfigurationChanges = ConfigChanges.Orientation | ConfigChanges.Keyboard | ConfigChanges.KeyboardHidden)]
    public class MainActivity : AndroidGameActivity
    {
        protected override void OnCreate(Bundle bundle)
        {
            base.OnCreate(bundle);

            Android.Graphics.Rect rect = new Android.Graphics.Rect();
            Window.DecorView.GetWindowVisibleDisplayFrame(rect);
            JitterPhysicsGame.PreferredSize = new Vector2(rect.Width(), rect.Height());

            var game = new JitterPhysicsGame();
            var view = (View)game.Services.GetService(typeof(View));
            SetContentView(view);
            game.Run();
        }
    }
}
