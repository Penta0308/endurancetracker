package kr.glora.endurancetracker

import android.annotation.SuppressLint
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothSocket
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CaptureRequest
import android.os.Bundle
import android.util.Log
import android.util.Range
import android.view.WindowManager
import androidx.annotation.OptIn
import androidx.appcompat.app.AppCompatActivity
import androidx.camera.camera2.internal.Camera2CameraInfoImpl
import androidx.camera.camera2.interop.Camera2Interop
import androidx.camera.camera2.interop.ExperimentalCamera2Interop
import androidx.camera.core.AspectRatio
import androidx.camera.core.CameraFilter
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.ImageCapture
import androidx.camera.core.Preview
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.core.content.ContextCompat
import com.psp.bluetoothlibrary.Bluetooth
import com.psp.bluetoothlibrary.BluetoothListener
import com.psp.bluetoothlibrary.BluetoothListener.onDevicePairListener
import com.psp.bluetoothlibrary.Connection
import kotlinx.coroutines.runBlocking
import kr.glora.endurancetracker.TrackOverlayView.Companion.FPS
import kr.glora.endurancetracker.databinding.ActivityMainBinding
import org.opencv.android.OpenCVLoader
import org.opencv.core.Point
import java.nio.charset.Charset
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors


typealias TrackListener = (found: Boolean, enduPos: Point, enduDir: Point, enduVel: Point, scalef: Float, m1: Int, m2: Int) -> Unit


class MainActivity : AppCompatActivity(), BluetoothListener.onConnectionListener, BluetoothListener.onReceiveListener {

    private lateinit var binding: ActivityMainBinding

    private lateinit var cameraExecutor: ExecutorService

    private lateinit var bluetooth: Bluetooth

    var connection: Connection? = null

    private var imageCapture: ImageCapture? = null

    private val deviceAddress = "98:D3:33:80:62:9D"

    @SuppressLint("RestrictedApi")
    @OptIn(ExperimentalCamera2Interop::class)
    fun startCamera() {
        val cameraProviderFuture = ProcessCameraProvider.getInstance(this)

        cameraProviderFuture.addListener({
            val cameraProvider: ProcessCameraProvider = cameraProviderFuture.get()

            val builder = Preview.Builder()

            val preview = builder
                .setTargetFrameRate(Range(29, 30))
                .setCameraSelector(CameraSelector.Builder()
                    .addCameraFilter { cameraInfos ->
                        // filter back cameras with minimum sensor pixel size
                        val backCameras = cameraInfos.filterIsInstance<Camera2CameraInfoImpl>()
                            .filter {
                                val pixelWidth =
                                    it.cameraCharacteristicsCompat.get(CameraCharacteristics.SENSOR_INFO_PIXEL_ARRAY_SIZE)?.width
                                        ?: 0
                                it.lensFacing == CameraSelector.LENS_FACING_BACK && pixelWidth >= 2000 // arbitrary number resolved empirically
                            }

                        // try to find wide lens camera, if not present, default to general backCameras
                        backCameras.minByOrNull {
                            val focalLengths =
                                it.cameraCharacteristicsCompat.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)
                            focalLengths?.getOrNull(0) ?: 0f
                        }
                            ?.let { listOf(it) } ?: backCameras
                    }.build())
                .build()
                .also {
                    it.setSurfaceProvider(binding.viewFinder.surfaceProvider)
                }

            imageCapture = ImageCapture.Builder().build()


            val imageAnalyzer = ImageAnalysis.Builder()
                .setTargetAspectRatio(AspectRatio.RATIO_4_3)
                .setImageQueueDepth(2)
                .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_RGBA_8888)
                .also {
                    val ext1 = Camera2Interop.Extender(it)
                        //.setCaptureRequestOption(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_OFF)
                        //.setCaptureRequestOption(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_OFF)
                        //.setCaptureRequestOption(CaptureRequest.LENS_FOCUS_DISTANCE, 1800.0F) // 1800 mm
                        //.setCaptureRequestOption(CaptureRequest.SENSOR_SENSITIVITY, 3200)
                        //.setCaptureRequestOption(CaptureRequest.SENSOR_EXPOSURE_TIME, 60) // 1/120 s
                        //.setCaptureRequestOption(CaptureRequest.SENSOR_FRAME_DURATION, 30) // 30 fps
                        .setCaptureRequestOption(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, Range(FPS.toInt(), FPS.toInt()))
                }
                .build()
                .also {
                    it.setAnalyzer(cameraExecutor, TrackDetectAnalyzer(this@MainActivity) { found, enduPos, enduDir, enduVel, scalef, m1, m2 ->
                        runOnUiThread {
                            binding.overlayView.updateEndu(found, enduPos, enduDir, enduVel, scalef)
                        }
                        runBlocking {
                            if (connection != null && connection!!.isConnected) {
                                if(m1 != 0 && m2 != 0) {
                                    val t = "f${m1},${m2}\n"
                                    Log.d("BTGuidance", t)
                                    connection!!.send(t.toByteArray(Charset.forName("ASCII")))
                                }
                            }
                        }
                    })}


            val cameraSelector = CameraSelector.DEFAULT_BACK_CAMERA

            try {
                cameraProvider.unbindAll()
                cameraProvider.bindToLifecycle(
                    this, cameraSelector, preview, imageCapture, imageAnalyzer)
            } catch(exc: Exception) {
                Log.e("CameraX", "Use case binding failed", exc)
            }



        }, ContextCompat.getMainExecutor(this))
    }

    @SuppressLint("MissingPermission")
    @OptIn(ExperimentalCamera2Interop::class)
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        if (OpenCVLoader.initLocal()) {
            Log.i("OpenCV", "OpenCV successfully loaded.")
        }

        bluetooth = Bluetooth(this)

        bluetooth.turnOnWithPermission(this)

        // Turn on bluetooth with user permission
        /* if you want to check user allow or denied bluetooth turn on
               request add override method onActivityResult and request code is Bluetooth.BLUETOOTH_ENABLE_REQUEST */
        bluetooth.turnOnWithPermission(this)

        bluetooth.setOnDetectNearbyDeviceListener { device -> // Device found
            Log.d("BT", "Device found " + device.name)
        }

        bluetooth.startDetectNearbyDevices()

        bluetooth.setOnDiscoveryStateChangedListener { state ->
            if (state == Bluetooth.DISCOVERY_STARTED) {
                Log.d("BT", "Discovery started")
            }
            if (state == Bluetooth.DISCOVERY_FINISHED) {
                Log.d("BT", "Discovery finished")
            }
        }


        // Listener
        bluetooth.setOnDevicePairListener(object : onDevicePairListener {
            override fun onDevicePaired(device: BluetoothDevice) {
                // Paired successful
                Log.d("BT", device.name + " Paired successful")
            }

            override fun onCancelled(device: BluetoothDevice) {
                // Pairing failed
                Log.d("BT", device.name + " Pairing failed")
            }
        })

        val devices = bluetooth.pairedDevices

        binding.btText.text.clear()
        //binding.btText.text.insert(0, devices.map { "${it.address} ${it.name}" }.joinToString(separator = "\n"))
        binding.btText.text.insert(0, deviceAddress)

        binding.btConnectButton.setOnClickListener {
            if (connection == null) {
                connection = Connection(this)
                connection!!.connect(binding.btText.text.toString(), false, this, this)
            }
        }


        cameraExecutor = Executors.newSingleThreadExecutor()
        startCamera()

        // Example of a call to a native method
        //binding.sampleText.text = stringFromJNI()

    }

    override fun onStop() {
        super.onStop()
        bluetooth.onStop()
    }

    override fun onDestroy() {
        super.onDestroy()
        cameraExecutor.shutdown()
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

    override fun onConnectionStateChanged(socket: BluetoothSocket?, state: Int) {
        Log.d("BTR", "onConnectionStateChanged: $socket $state")
    }

    override fun onConnectionFailed(errorCode: Int) {
        Log.d("BTR", "onConnectionFailed: $errorCode")
    }

    override fun onReceived(receivedData: String?) {
        Log.d("BTR", "onReceived: $receivedData")
    }
}