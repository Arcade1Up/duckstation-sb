package umido.androidemu.mame;

import android.content.Context;
import android.content.res.AssetManager;
import android.os.Environment;
import android.util.Log;
import android.view.Surface;
import android.widget.Toast;

import java.io.IOException;
import java.io.InputStream;
import java.util.Locale;

public class AndroidHostInterface {
    public final static int DISPLAY_ALIGNMENT_TOP_OR_LEFT = 0;
    public final static int DISPLAY_ALIGNMENT_CENTER = 1;
    public final static int DISPLAY_ALIGNMENT_RIGHT_OR_BOTTOM = 2;

    private long mNativePointer;
    private Context mContext;

    static public native String getScmVersion();
    static public native String getFullScmVersion();

    static public native AndroidHostInterface create(Context context, String userDirectory);

    public AndroidHostInterface(Context context) {
        this.mContext = context;
    }

    public void reportError(String message) {
        //Toast.makeText(mContext, message, Toast.LENGTH_LONG).show();
    }

    public void reportMessage(String message) {
        //Toast.makeText(mContext, message, Toast.LENGTH_SHORT).show();
    }

    public InputStream openAssetStream(String path) {
        try {
            return mContext.getAssets().open(path, AssetManager.ACCESS_STREAMING);
        } catch (IOException e) {
            return null;
        }
    }

    public native boolean isEmulationThreadRunning();

    public native boolean startEmulationThread(EmulationActivity emulationActivity, Surface surface, String filename, boolean resumeState, String state_filename);

    public native boolean isEmulationThreadPaused();

    public native void pauseEmulationThread(boolean paused);

    public native void stopEmulationThread();

    public native void shutdown();

    public native boolean hasSurface();

    public native void surfaceChanged(Surface surface, int format, int width, int height);

    // TODO: Find a better place for this.
    public native void setControllerType(int index, String typeName);

    public native void setControllerButtonState(int index, int buttonCode, boolean pressed);

    public native void setControllerAxisState(int index, int axisCode, float value);

    public static native int getControllerButtonCode(String controllerType, String buttonName);

    public static native int getControllerAxisCode(String controllerType, String axisName);

    public native void refreshGameList(boolean invalidateCache, boolean invalidateDatabase, AndroidProgressCallback progressCallback);

    public native GameListEntry[] getGameListEntries();

    public native void resetSystem();

    public native void loadState(boolean global, int slot);

    //public native void saveState(boolean global, int slot);

    public native void saveResumeState(boolean waitForCompletion);

    public native void applySettings();

    public native void setDisplayAlignment(int alignment);

    public native PatchCode[] getPatchCodeList();

    public native void setPatchCodeEnabled(int index, boolean enabled);

    public native boolean importPatchCodesFromString(String str);

    //public native void addOSDMessage(String message, float duration);

    public native boolean hasAnyBIOSImages();

    public native String importBIOSImage(byte[] data);

    public native boolean isFastForwardEnabled();

    public native void setFastForwardEnabled(boolean enabled);

    public native String[] getMediaPlaylistPaths();

    public native int getMediaPlaylistIndex();

    public native boolean setMediaPlaylistIndex(int index);

    static {
        System.loadLibrary("arcade1up-native");
    }

    static private AndroidHostInterface mInstance;

    static public boolean createInstance(Context context) {
        // Set user path.
        String externalStorageDirectory = Environment.getExternalStorageDirectory().getAbsolutePath();
        if (externalStorageDirectory.isEmpty())
            externalStorageDirectory = "/sdcard";

        externalStorageDirectory += "/arcade1up";
        Log.i("AndroidHostInterface", "User directory: " + externalStorageDirectory);
        mInstance = create(context, externalStorageDirectory);
        return mInstance != null;
    }

    static public boolean hasInstance() {
        return mInstance != null;
    }

    static public AndroidHostInterface getInstance() {
        return mInstance;
    }

    static public boolean hasInstanceAndEmulationThreadIsRunning() {
        return hasInstance() && getInstance().isEmulationThreadRunning();
    }
}
