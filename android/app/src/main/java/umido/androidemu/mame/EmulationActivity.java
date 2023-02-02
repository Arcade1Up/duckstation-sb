package umido.androidemu.mame;

import android.app.Dialog;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.ActivityInfo;
import android.content.res.Configuration;
import android.hardware.input.InputManager;
import android.net.Uri;
import android.os.Bundle;
import android.os.Vibrator;
import android.util.Log;
import android.view.KeyEvent;
import android.view.SurfaceHolder;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.FrameLayout;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;
import androidx.preference.PreferenceManager;

import static android.view.KeyEvent.ACTION_UP;
import static android.view.KeyEvent.KEYCODE_BUTTON_SELECT;
import static android.view.KeyEvent.KEYCODE_ENTER;

/**
 * An example full-screen activity that shows and hides the system UI (i.e.
 * status bar and navigation/system bar) with user intesraction.
 */
public class EmulationActivity extends AppCompatActivity implements SurfaceHolder.Callback {
    /**
     * Settings interfaces.
     */
    private SharedPreferences mPreferences;
    private boolean mWasDestroyed = false;
    private boolean mStopRequested = false;
    private boolean mApplySettingsOnSurfaceRestored = false;
    private String mGameTitle = null;
    private EmulationSurfaceView mContentView;
    private int mSaveStateSlot = 0;

    private boolean getBooleanSetting(String key, boolean defaultValue) {
        return mPreferences.getBoolean(key, defaultValue);
    }

    private void setBooleanSetting(String key, boolean value) {
        SharedPreferences.Editor editor = mPreferences.edit();
        editor.putBoolean(key, value);
        editor.apply();
    }

    private String getStringSetting(String key, String defaultValue) {
        return mPreferences.getString(key, defaultValue);
    }

    private void setStringSetting(String key, String value) {
        SharedPreferences.Editor editor = mPreferences.edit();
        editor.putString(key, value);
        editor.apply();
    }

    private void reportErrorOnUIThread(String message) {
        // Toast.makeText(this, message, Toast.LENGTH_LONG);
        new AlertDialog.Builder(this)
                .setTitle(R.string.emulation_activity_error)
                .setMessage(message)
                .setPositiveButton(R.string.emulation_activity_ok, (dialog, button) -> {
                    dialog.dismiss();
                    enableFullscreenImmersive();
                })
                .create()
                .show();
    }

    public void reportError(String message) {
        Log.e("EmulationActivity", message);

        Object lock = new Object();
        runOnUiThread(() -> {
            // Toast.makeText(this, message, Toast.LENGTH_LONG);
            new AlertDialog.Builder(this)
                    .setTitle(R.string.emulation_activity_error)
                    .setMessage(message)
                    .setPositiveButton(R.string.emulation_activity_ok, (dialog, button) -> {
                        dialog.dismiss();
                        enableFullscreenImmersive();
                        synchronized (lock) {
                            lock.notify();
                        }
                    })
                    .create()
                    .show();
        });

        synchronized (lock) {
            try {
                lock.wait();
            } catch (InterruptedException e) {
            }
        }
    }

    public void reportMessage(String message) {
        Log.i("EmulationActivity", message);
        /*runOnUiThread(() -> {
            Toast.makeText(this, message, Toast.LENGTH_SHORT);
        });*/
    }

    public void onEmulationStarted() {
    }

    public void onEmulationStopped() {
        runOnUiThread(() -> {
            AndroidHostInterface.getInstance().stopEmulationThread();
            if (!mWasDestroyed && !mStopRequested)
                finish();
        });
    }

    public void onGameTitleChanged(String title) {
        runOnUiThread(() -> {
            mGameTitle = title;
        });
    }

    private void doApplySettings() {
        AndroidHostInterface.getInstance().applySettings();
        updateRequestedOrientation();
        updateControllers();
    }

    private void applySettings() {
        if (!AndroidHostInterface.getInstance().isEmulationThreadRunning())
            return;

        if (AndroidHostInterface.getInstance().hasSurface()) {
            doApplySettings();
        } else {
            mApplySettingsOnSurfaceRestored = true;
        }
    }

    /// Ends the activity if it was restored without properly being created.
    private boolean checkActivityIsValid() {
        if (!AndroidHostInterface.hasInstance() || !AndroidHostInterface.getInstance().isEmulationThreadRunning()) {
            finish();
            return false;
        }

        return true;
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
        // Once we get a surface, we can boot.
        if (AndroidHostInterface.getInstance().isEmulationThreadRunning()) {
            AndroidHostInterface.getInstance().surfaceChanged(holder.getSurface(), format, width, height);
            updateOrientation();

            if (mApplySettingsOnSurfaceRestored) {
                mApplySettingsOnSurfaceRestored = false;
                doApplySettings();
            }

            return;
        }

        final String bootPath = getIntent().getStringExtra("bootPath");
        final boolean resumeState = getIntent().getBooleanExtra("resumeState", false);
        final String bootSaveStatePath = getIntent().getStringExtra("saveStatePath");
        final boolean attractMode = getIntent().getBooleanExtra("attract", false);
        mContentView.setAttractMode(attractMode);
        AndroidHostInterface.getInstance().startEmulationThread(this, holder.getSurface(), bootPath, resumeState, bootSaveStatePath);
        updateRequestedOrientation();
        updateOrientation();
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
        if (!AndroidHostInterface.getInstance().isEmulationThreadRunning())
            return;

        Log.i("EmulationActivity", "Surface destroyed");

        // Save the resume state in case we never get back again...
        if (!mStopRequested)
            AndroidHostInterface.getInstance().saveResumeState(true);

        AndroidHostInterface.getInstance().surfaceChanged(null, 0, 0, 0);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        mPreferences = PreferenceManager.getDefaultSharedPreferences(this);
        Log.i("EmulationActivity", "OnCreate");

        // we might be coming from a third-party launcher if the host interface isn't setup
        if (!AndroidHostInterface.hasInstance() && !AndroidHostInterface.createInstance(this)) {
            finish();
            return;
        }
        supportRequestWindowFeature(Window.FEATURE_NO_TITLE);
        enableFullscreenImmersive();
        setContentView(R.layout.activity_emulation);

        mContentView = findViewById(R.id.fullscreen_content);
        mContentView.getHolder().addCallback(this);
        mContentView.setFocusable(true);
        mContentView.requestFocus();

        SFX.InitSFX(this);

        // Hook up controller input.
        updateControllers();
        registerInputDeviceListener();
    }

    @Override
    protected void onPostCreate(Bundle savedInstanceState) {
        super.onPostCreate(savedInstanceState);
        enableFullscreenImmersive();
    }

    @Override
    protected void onPostResume() {
        super.onPostResume();
        enableFullscreenImmersive();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        Log.i("EmulationActivity", "OnStop");
        if (AndroidHostInterface.getInstance().isEmulationThreadRunning()) {
            mWasDestroyed = true;
            AndroidHostInterface.getInstance().stopEmulationThread();
        }

        unregisterInputDeviceListener();
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, @Nullable Intent data) {
        super.onActivityResult(requestCode, resultCode, data);

        if (!checkActivityIsValid()) {
            // we must've got killed off in the background :(
            return;
        }

        if (requestCode == REQUEST_CODE_SETTINGS) {
            if (AndroidHostInterface.getInstance().isEmulationThreadRunning()) {
                applySettings();
            }
        } else if (requestCode == REQUEST_IMPORT_PATCH_CODES) {
            if (data != null)
                importPatchesFromFile(data.getData());
        }
    }

    @Override
    public boolean dispatchKeyEvent(KeyEvent event) {
        if (event.getKeyCode() == KEYCODE_BUTTON_SELECT /*KEYCODE_ENTER */ && event.getAction() == ACTION_UP ) {//
            if (!AndroidHostInterface.getInstance().isEmulationThreadPaused())
                AndroidHostInterface.getInstance().pauseEmulationThread(true);
            SFX.notifySFX();
            SFX.playClick();
            showConfirmation();
            return true;
        }
        return super.dispatchKeyEvent(event);
    }

    private void showConfirmation(){
        Log.i("ConfirmationDialog", "Showing confirmation popup");
        Dialog confirmation = new ConfirmationDialog(this);
        confirmation.show();
    }

    @Override
    public void onBackPressed() {
        showMenu();
    }

    @Override
    public void onConfigurationChanged(@NonNull Configuration newConfig) {
        super.onConfigurationChanged(newConfig);

        if (checkActivityIsValid())
            updateOrientation(newConfig.orientation);
    }

    private void updateRequestedOrientation() {
        final String orientation = getStringSetting("Main/EmulationScreenOrientation", "unspecified");
        if (orientation.equals("portrait"))
            setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_USER_PORTRAIT);
        else if (orientation.equals("landscape"))
            setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_USER_LANDSCAPE);
        else
            setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_UNSPECIFIED);
    }

    private void updateOrientation() {
        final int orientation = getResources().getConfiguration().orientation;
        updateOrientation(orientation);
    }

    private void updateOrientation(int newOrientation) {
        if (!AndroidHostInterface.getInstance().isEmulationThreadRunning())
            return;

        if (newOrientation == Configuration.ORIENTATION_PORTRAIT)
            AndroidHostInterface.getInstance().setDisplayAlignment(AndroidHostInterface.DISPLAY_ALIGNMENT_TOP_OR_LEFT);
        else
            AndroidHostInterface.getInstance().setDisplayAlignment(AndroidHostInterface.DISPLAY_ALIGNMENT_CENTER);
    }

    private void enableFullscreenImmersive() {
        this.getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,WindowManager.LayoutParams.FLAG_FULLSCREEN);

        getWindow().getDecorView().setSystemUiVisibility(
                View.SYSTEM_UI_FLAG_LAYOUT_STABLE |
                        View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION |
                        View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN |
                        View.SYSTEM_UI_FLAG_HIDE_NAVIGATION |
                        View.SYSTEM_UI_FLAG_FULLSCREEN |
                        View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY);
        if (mContentView != null)
            mContentView.requestFocus();
    }

    private static final int REQUEST_CODE_SETTINGS = 0;
    private static final int REQUEST_IMPORT_PATCH_CODES = 1;

    private void onMenuClosed() {
        enableFullscreenImmersive();

        if (AndroidHostInterface.getInstance().isEmulationThreadPaused())
            AndroidHostInterface.getInstance().pauseEmulationThread(false);
    }

    private void showMenu() {
        if (getBooleanSetting("Main/PauseOnMenu", false) &&
                !AndroidHostInterface.getInstance().isEmulationThreadPaused()) {
            AndroidHostInterface.getInstance().pauseEmulationThread(true);
        }

        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setItems(R.array.emulation_menu, (dialogInterface, i) -> {
            switch (i) {
                case 0:     // Quick Load
                {
                    AndroidHostInterface.getInstance().loadState(false, mSaveStateSlot);
                    onMenuClosed();
                    return;
                }

                case 1:     // Quick Save
                {
                    //AndroidHostInterface.getInstance().saveState(false, mSaveStateSlot);
                    onMenuClosed();
                    return;
                }

                case 2:     // Save State Slot
                {
                    showSaveStateSlotMenu();
                    return;
                }

                case 3:     // Toggle Fast Forward
                {
                    AndroidHostInterface.getInstance().setFastForwardEnabled(!AndroidHostInterface.getInstance().isFastForwardEnabled());
                    onMenuClosed();
                    return;
                }

                case 4:     // More Options
                {
                    showMoreMenu();
                    return;
                }

                case 5:     // Quit
                {
                    mStopRequested = true;
                    AndroidHostInterface.getInstance().stopEmulationThread();
                    AndroidHostInterface.getInstance().shutdown();
                    finish();
                    return;
                }
            }
        });
        builder.setOnCancelListener(dialogInterface -> onMenuClosed());
        builder.create().show();
    }

    private void showSaveStateSlotMenu() {
        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setSingleChoiceItems(R.array.emulation_save_state_slot_menu, mSaveStateSlot, (dialogInterface, i) -> {
            mSaveStateSlot = i;
            dialogInterface.dismiss();
            onMenuClosed();
        });
        builder.setOnCancelListener(dialogInterface -> onMenuClosed());
        builder.create().show();
    }

    private void showMoreMenu() {
        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        if (mGameTitle != null && !mGameTitle.isEmpty())
            builder.setTitle(mGameTitle);

        builder.setItems(R.array.emulation_more_menu, (dialogInterface, i) -> {
            switch (i) {
                case 0:     // Reset
                {
                    AndroidHostInterface.getInstance().resetSystem();
                    onMenuClosed();
                    return;
                }

                case 1:     // Patch Codes
                {
                    showPatchesMenu();
                    return;
                }

                case 2:     // Change Disc
                {
                    showDiscChangeMenu();
                    return;
                }

                case 3:     // Change Touchscreen Controller
                {
                    showTouchscreenControllerMenu();
                    return;
                }

                case 4:     // Settings
                {
                    Intent intent = new Intent(EmulationActivity.this, SettingsActivity.class);
                    intent.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP);
                    startActivityForResult(intent, REQUEST_CODE_SETTINGS);
                    return;
                }
            }
        });
        builder.setOnCancelListener(dialogInterface -> onMenuClosed());
        builder.create().show();
    }

    private void showTouchscreenControllerMenu() {
        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setItems(R.array.settings_touchscreen_controller_view_entries, (dialogInterface, i) -> {
            String[] values = getResources().getStringArray(R.array.settings_touchscreen_controller_view_values);
            setStringSetting("Controller1/TouchscreenControllerView", values[i]);
            updateControllers();
            onMenuClosed();
        });
        builder.setOnCancelListener(dialogInterface -> onMenuClosed());
        builder.create().show();
    }

    private void showPatchesMenu() {
        final PatchCode[] codes = AndroidHostInterface.getInstance().getPatchCodeList();

        AlertDialog.Builder builder = new AlertDialog.Builder(this);

        CharSequence[] items = new CharSequence[(codes != null) ? (codes.length + 1) : 1];
        items[0] = getString(R.string.emulation_activity_import_patch_codes);
        if (codes != null) {
            for (int i = 0; i < codes.length; i++) {
                final PatchCode cc = codes[i];
                items[i + 1] = String.format("%s %s", cc.isEnabled() ? getString(R.string.emulation_activity_patch_on) : getString(R.string.emulation_activity_patch_off), cc.getDescription());
            }
        }

        builder.setItems(items, (dialogInterface, i) -> {
            if (i > 0) {
                AndroidHostInterface.getInstance().setPatchCodeEnabled(i - 1, !codes[i - 1].isEnabled());
                onMenuClosed();
            } else {
                Intent intent = new Intent(Intent.ACTION_GET_CONTENT);
                intent.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP);
                intent.setType("*/*");
                intent.addCategory(Intent.CATEGORY_OPENABLE);
                startActivityForResult(Intent.createChooser(intent, getString(R.string.emulation_activity_choose_patch_code_file)), REQUEST_IMPORT_PATCH_CODES);
            }
        });
        builder.setOnCancelListener(dialogInterface -> onMenuClosed());
        builder.create().show();
    }

    private void importPatchesFromFile(Uri uri) {
        String str = FileUtil.readFileFromUri(this, uri, 512 * 1024);
        if (str == null || !AndroidHostInterface.getInstance().importPatchCodesFromString(str)) {
            reportErrorOnUIThread(getString(R.string.emulation_activity_failed_to_import_patch_codes));
        }
    }

    private void showDiscChangeMenu() {
        final String[] paths = AndroidHostInterface.getInstance().getMediaPlaylistPaths();
        final int currentPath = AndroidHostInterface.getInstance().getMediaPlaylistIndex();
        if (paths == null) {
            onMenuClosed();
            return;
        }

        AlertDialog.Builder builder = new AlertDialog.Builder(this);

        CharSequence[] items = new CharSequence[paths.length];
        for (int i = 0; i < paths.length; i++)
            items[i] = GameListEntry.getFileNameForPath(paths[i]);

        builder.setSingleChoiceItems(items, currentPath, (dialogInterface, i) -> {
            AndroidHostInterface.getInstance().setMediaPlaylistIndex(i);
            dialogInterface.dismiss();
            onMenuClosed();
        });
        builder.setOnCancelListener(dialogInterface -> onMenuClosed());
        builder.create().show();
    }

    /**
     * Touchscreen controller overlay
     */
    TouchscreenControllerView mTouchscreenController;

    public void updateControllers() {
        final String controllerType = getStringSetting("Controller1/Type", "AnalogController");
        final String viewType = getStringSetting("Controller1/TouchscreenControllerView", "digital");
        final boolean autoHideTouchscreenController = getBooleanSetting("Controller1/AutoHideTouchscreenController", true);
        final boolean hapticFeedback = getBooleanSetting("Controller1/HapticFeedback", false);
        final boolean vibration = getBooleanSetting("Controller1/Vibration", false);
        final FrameLayout activityLayout = findViewById(R.id.frameLayout);

        Log.i("EmulationActivity", "Controller type: " + controllerType);
        Log.i("EmulationActivity", "View type: " + viewType);

        final boolean hasAnyControllers = mContentView.initControllerMapping(controllerType);

        if (controllerType == "none" || viewType == "none" || (hasAnyControllers && autoHideTouchscreenController)) {
            if (mTouchscreenController != null) {
                activityLayout.removeView(mTouchscreenController);
                mTouchscreenController = null;
                mVibratorService = null;
            }
        } else {
            if (mTouchscreenController == null) {
                mTouchscreenController = new TouchscreenControllerView(this);
                if (vibration)
                    mVibratorService = (Vibrator) getSystemService(VIBRATOR_SERVICE);

                activityLayout.addView(mTouchscreenController);
            }

            mTouchscreenController.init(0, controllerType, viewType, hapticFeedback);
        }
    }

    private InputManager.InputDeviceListener mInputDeviceListener;

    private void registerInputDeviceListener() {
        if (mInputDeviceListener != null)
            return;

        mInputDeviceListener = new InputManager.InputDeviceListener() {
            @Override
            public void onInputDeviceAdded(int i) {
                Log.i("EmulationActivity", String.format("InputDeviceAdded %d", i));
                updateControllers();
            }

            @Override
            public void onInputDeviceRemoved(int i) {
                Log.i("EmulationActivity", String.format("InputDeviceRemoved %d", i));
                updateControllers();
            }

            @Override
            public void onInputDeviceChanged(int i) {
                Log.i("EmulationActivity", String.format("InputDeviceChanged %d", i));
                updateControllers();
            }
        };

        InputManager inputManager = ((InputManager) getSystemService(Context.INPUT_SERVICE));
        if (inputManager != null)
            inputManager.registerInputDeviceListener(mInputDeviceListener, null);
    }

    private void unregisterInputDeviceListener() {
        if (mInputDeviceListener == null)
            return;

        InputManager inputManager = ((InputManager) getSystemService(Context.INPUT_SERVICE));
        if (inputManager != null)
            inputManager.unregisterInputDeviceListener(mInputDeviceListener);

        mInputDeviceListener = null;
    }

    private Vibrator mVibratorService;

    public void setVibration(boolean enabled) {
        if (mVibratorService == null)
            return;

        runOnUiThread(() -> {
            if (mVibratorService == null)
                return;

            if (enabled)
                mVibratorService.vibrate(1000);
            else
                mVibratorService.cancel();
        });
    }
}
