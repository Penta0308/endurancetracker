package kr.glora.endurancetracker

import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.util.Log
import android.view.SurfaceView
import android.view.WindowManager
import android.widget.TextView
import androidx.loader.app.LoaderManager.LoaderCallbacks
import dalvik.system.BaseDexClassLoader
import kr.glora.endurancetracker.databinding.ActivityMainBinding
import org.opencv.android.CameraBridgeViewBase
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2
import org.opencv.android.OpenCVLoader
import org.opencv.core.Mat
import org.opencv.core.MatOfKeyPoint
import org.opencv.imgproc.Imgproc
import org.opencv.objdetect.ArucoDetector
import org.opencv.objdetect.Objdetect
import org.opencv.objdetect.Objdetect.DICT_ARUCO_ORIGINAL

class MainActivity : AppCompatActivity(), CvCameraViewListener2 {

    private lateinit var binding: ActivityMainBinding

    private val arucoMarkerTag = 45

    private var mRgba: Mat? = null
    //private var arucoMarker: Mat? = null
    private var arucoDetector: ArucoDetector? = null

    override fun onPause() {
        binding.frameSurface.disableView()
        super.onPause()
    }

    override fun onResume() {
        super.onResume()
        binding.frameSurface.enableView()
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        // Example of a call to a native method
        //binding.sampleText.text = stringFromJNI()

        if (OpenCVLoader.initLocal()) {
            Log.i("OpenCV", "OpenCV successfully loaded.");
        }

        //Objdetect.generateImageMarker(Objdetect.getPredefinedDictionary(DICT_ARUCO_ORIGINAL), arucoMarkerTag, 200, arucoMarker, 1)
        arucoDetector = ArucoDetector(Objdetect.getPredefinedDictionary(DICT_ARUCO_ORIGINAL))

        binding.frameSurface.setCameraIndex(0)
        binding.frameSurface.setCvCameraViewListener(this)
        binding.frameSurface.setCameraPermissionGranted()
        Log.i("OpenCV", "Camera successfully loaded.");
        binding.frameSurface.enableView()
        binding.frameSurface.enableFpsMeter()
        binding.frameSurface.visibility = SurfaceView.VISIBLE

    }

    /**
     * A native method that is implemented by the 'endurancetracker' native library,
     * which is packaged with this application.
     */
    //external fun stringFromJNI(): String
    external fun detPos(matNativeObjAddr: Long): DoubleArray

    companion object {
        // Used to load the 'endurancetracker' library on application startup.
        init {
            System.loadLibrary("opencv_java4")
            System.loadLibrary("endurancetracker")
        }
    }

    override fun onCameraViewStarted(width: Int, height: Int) { }

    override fun onCameraViewStopped() { }

    override fun onCameraFrame(inputFrame: CameraBridgeViewBase.CvCameraViewFrame?): Mat? {
        mRgba = inputFrame?.rgba()

        if (mRgba == null) return mRgba

        val lMconors = ArrayList<Mat>(1)
        val mMids = Mat()

        arucoDetector!!.detectMarkers(mRgba, lMconors, mMids)

        val mD: Mat = Mat()
        Imgproc.cvtColor(mRgba!!, mD, Imgproc.COLOR_RGBA2RGB)

        Objdetect.drawDetectedMarkers(mD, lMconors, mMids)

        return mD
    }
}