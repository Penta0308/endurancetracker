package kr.glora.endurancetracker

import android.annotation.SuppressLint
import android.content.Context
import android.util.Log
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.ImageProxy
import kr.glora.endurancetracker.TrackOverlayView.Companion.FPS
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.imgproc.Imgproc
import org.opencv.objdetect.ArucoDetector
import org.opencv.objdetect.Objdetect
import org.opencv.objdetect.Objdetect.DICT_4X4_50
import org.opencv.video.KalmanFilter
import java.nio.ByteBuffer
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.roundToInt
import kotlin.math.sign
import kotlin.math.sin
import kotlin.math.sqrt

class TrackDetectAnalyzer(
    context: Context,
    private val listener: TrackListener
) : ImageAnalysis.Analyzer {
    var orbCenterForce: Double = 0.00001

    private val arucoMarkerTag = 2
    private val arucoDetector = ArucoDetector(Objdetect.getPredefinedDictionary(DICT_4X4_50))
    
    //private val markerDir = 1.5
    private val markerDir = 0.0

    private val peDist = 0.15

    private val peVel = 0.05

    //private val apDist
    //    get() = (sqrt(GM * GM + peVel * peVel * peDist * peDist * (peVel * peVel - 2 * GM / peDist)) - GM) / (peVel * peVel - 2 * GM / peDist)
    private val apDist = peDist
    private val apVel = peVel

    //private val apVel
    //    get() = L / apDist

    private val GM
        get() = 0.0003 // 원운동: peVel * peDist ^ 2, 현재 타원.
    private val L: Double
        get() = peVel * peDist

    private val semiMajorAxis = (apDist + peDist) / 2
    private val semiMinorAxis = sqrt(semiMajorAxis * semiMajorAxis - (apDist - peDist) * (apDist - peDist))

    private val e
        get() = sqrt(1.0 - semiMinorAxis * semiMinorAxis / semiMajorAxis / semiMajorAxis)

    private val motorScale = 2048.0


    private fun theta2tgtr(theta: Double): Double {
        //return L * L / GM / (1 + e * cos(theta))
        return peDist
    }

    private val mControl = Mat(4, 1, CvType.CV_32FC1).apply {
        put(0, 0, 0.0, 0.0, 0.0, 0.0)
    }

    private val mKalmanFilter = KalmanFilter(8, 4, 4).apply {
        this._processNoiseCov.also { it.put(0, 0,
            0.001, 0.0, 0.0, 0.0, 0.000001, 0.0, 0.0, 0.0,
            0.0, 0.001, 0.0, 0.0, 0.0, 0.000001, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.000001, 0.0, 0.0, 0.0, 0.002, 0.001, 0.0001, 0.0001,
            0.0, 0.000001, 0.0, 0.0, 0.001, 0.002, 0.0001, 0.0001,
            0.0, 0.0, 0.0, 0.0, 0.0001, 0.0001, 0.002, 0.0001,
            0.0, 0.0, 0.0, 0.0, 0.0001, 0.0001, 0.0001, 0.002)
        }

        this._measurementNoiseCov.also {
            it.put(0, 0,
                0.02, 0.01, 0.01, 0.01,
                0.01, 0.02, 0.01, 0.01,
                0.01, 0.01, 0.06, 0.01,
                0.01, 0.01, 0.01, 0.06
            )
        }

        this._controlMatrix.also {
            it.put(0, 0,
                0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0,
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0,
            )
        }

        /*for (x in 0..<this._errorCovPost.width()) {
            for (y in 0..<this._errorCovPost.height()) {
                this._errorCovPost.put(x, y, if (x == y) 1e4 else 0.0)
                this._errorCovPre.put(x, y, if (x == y) 1e4 else 0.0)
            }
        }*/

        this._transitionMatrix.also { it.put(0, 0,
            1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0
        )}
    }

    private val kalmanH = Mat(4, 8, CvType.CV_32FC1).also { it.put(0, 0,
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0
    )}

    private val kalmanHinvalid = Mat(4, 8, CvType.CV_32FC1).also { it.put(0, 0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    )}

    init {
        //Objdetect.generateImageMarker(Objdetect.getPredefinedDictionary(DICT_ARUCO_ORIGINAL), arucoMarkerTag, 200, arucoMarker, 1)
    }

    private fun ByteBuffer.toByteArray(): ByteArray {
        rewind()    // Rewind the buffer to zero
        val data = ByteArray(remaining())
        get(data)   // Copy the buffer into a byte array
        return data // Return the byte array
    }

    @SuppressLint("UnsafeOptInUsageError")
    override fun analyze(imageProxy: ImageProxy) {
        val buffer = imageProxy.planes[0].buffer // Taking Y Plane of Image

        //Log.d("EnduTrack", imageProxy.width.toString())
        //Log.d("EnduTrack", imageProxy.height.toString())



        val yData = buffer.toByteArray()
        val yMat = Mat(imageProxy.height, imageProxy.width, CvType.CV_8UC4)
        yMat.put(0, 0, yData)

        val tyMat = Mat()
        Imgproc.cvtColor(yMat, tyMat, Imgproc.COLOR_RGBA2BGR)
        yMat.release()
        //val tyMat = cMat.t()
        //cMat.release()
        //val tyMat = yMat.t()

        /*
        val TAG = "EnduTrack"

        Log.d(TAG, "Mat type: ${tyMat?.type()}")
        Log.d(TAG, "Mat size: ${tyMat?.size()}")
        Log.d(TAG, "Mat channels: ${tyMat?.channels()}")
        Log.d(TAG, "Mat depth: ${tyMat?.depth()}")
        */

        val lMcorners = ArrayList<Mat>(1)
        val mMids = Mat()

        arucoDetector.detectMarkers(tyMat, lMcorners, mMids)
        //Log.d("CVAruco", "Detected ${lMcorners.size} marke
        val mmP = Mat(4, 1, CvType.CV_32FC1)
        mmP.put(0, 0, 0.0, 0.0, 0.0, 0.0)

        var found = false

        if (lMcorners.size >= 1) {
            for (i in 0..<mMids.height()) {
                val tag = mMids.get(i, 0)[0].roundToInt()
                val lu = mapToCenter(lMcorners[i].get(0, 0), tyMat)
                val ru = mapToCenter(lMcorners[i].get(0, 1), tyMat)
                val rd = mapToCenter(lMcorners[i].get(0, 2), tyMat)
                val ld = mapToCenter(lMcorners[i].get(0, 3), tyMat)

                val areaSq = (((lu[0] - ru[0]).pow(2.0) + (lu[1] - ru[1]).pow(2.0)) * ((lu[0] - ru[0]).pow(
                    2.0
                ) + (ld[1] - ld[1]).pow(2.0))).pow(0.5)

                if (areaSq > 0.0001 && tag == arucoMarkerTag) {
                    val mx = (lu[0] + ld[0] + ru[0] + rd[0]) / 4
                    val my = (lu[1] + ld[1] + ru[1] + rd[1]) / 4
                    val _dirx = (lu[0] - ld[0] + ru[0] - rd[0]) / 2
                    val _diry = (lu[1] - ld[1] + ru[1] - rd[1]) / 2
                    val dirx = _dirx * cos(markerDir) - _diry * sin(markerDir)
                    val diry = _dirx * sin(markerDir) + _diry * cos(markerDir)

                    mmP.put(0, 0, mx, my, dirx, diry)

                    Log.d("CVAruco${tag}", "At ($mx, $my, $dirx, $diry)")
                    //Objdetect.drawDetectedMarkers(tyMat, lMcorners.subList(i, i+1), mMids.submat(Range(i, i), Range.all()))
                    found = true
                }
            }
        }

        if (found) mKalmanFilter._measurementMatrix = kalmanH
        else mKalmanFilter._measurementMatrix = kalmanHinvalid

        mKalmanFilter.predict(mControl)

        val mp = mKalmanFilter._statePost

        // mControl : delta of PosXvel PosYvel DirXvel DirYvel

        val orbCurR = sqrt(mp.get(0, 0)[0].pow(2) + mp.get(1, 0)[0].pow(2))

        val orbDir = sign(mp.get(0, 0)[0] * mp.get(5, 0)[0] - mp.get(1, 0)[0] * mp.get(4, 0)[0]).let { if(it == 0.0) 1.0 else it }

        /*if (orbCurR < 0.001) mControl.put(0, 0, - mp.get(0, 0)[0] / orbCurR.pow(3) / FPS * orbCenterForce, - mp.get(1, 0)[0] / orbCurR.pow(3) / FPS * orbCenterForce)
        else*/ mControl.put(0, 0, 0.0, 0.0, 0.0, 0.0)

        Log.d("CVKalman", "(${mp.get(0, 0)[0]}, ${mp.get(1, 0)[0]}, ${mp.get(4, 0)[0]}, ${mp.get(5, 0)[0]})")

        var m1: Int = 0
        var m2: Int = 0

        if (orbCurR > 0.10) {
            val curTheta = atan2(mp.get(0, 0)[0], mp.get(1, 0)[0])
            val tgtR = theta2tgtr(curTheta)
            val tgtRd = theta2tgtr(atan2(mp.get(0, 0)[0], mp.get(1, 0)[0]) + 0.001 * orbDir)
            var tgtV = sqrt((tgtRd - tgtR) * (tgtRd - tgtR) / 0.001 / 0.001)

            Log.d("OrbTgt", "tgtR: ${tgtR}, curR: ${orbCurR}, curTheta: ${curTheta}")

            var movPosTheta = L / tgtR

            val movPosR = (tgtR - orbCurR)

            val tgtVelX =
                mp.get(0, 0)[0] / orbCurR * movPosR - mp.get(1, 0)[0] / orbCurR * movPosTheta
            val tgtVelY =
                mp.get(1, 0)[0] / orbCurR * movPosR + mp.get(0, 0)[0] / orbCurR * movPosTheta

            val tgtVelSize = sqrt(tgtVelX * tgtVelX + tgtVelY * tgtVelY) * peDist / orbCurR
            //val headingCorrectionX = tgtVelX - mp.get(4, 0)[0]
            //val headingCorrectionY = tgtVelY - mp.get(5, 0)[0]

            val curHeading = atan2(mp.get(3, 0)[0], mp.get(2, 0)[0])

            val tgtHeading = atan2(tgtVelY, tgtVelX)

            var hdgDiff = curHeading - tgtHeading - PI

            if(hdgDiff > PI) hdgDiff -= 2 * PI
            if(hdgDiff < -PI) hdgDiff += 2 * PI
            if(hdgDiff > PI) hdgDiff -= 2 * PI
            if(hdgDiff < -PI) hdgDiff += 2 * PI

            Log.d("OrbMot", "tgtVelX: ${tgtVelX}, tgtVelY: ${tgtVelY}, tgtHeading: ${tgtHeading} movPosR: ${movPosR}, hdgDiff: ${hdgDiff}")

            //val rotLim =

            m1 = min(max(((tgtVelSize - max(min(movPosR * 0.5, 0.01), -0.01) + max(min(hdgDiff * 0.6, 0.01), -0.01)) * motorScale).roundToInt(), 0), 255)
            // Outer
            m2 = min(max(((tgtVelSize * 0.85 + max(min(movPosR * 0.5, 0.01), -0.01) - max(min(hdgDiff * 0.6, 0.01), -0.01)) * motorScale).roundToInt(), 0), 255)
            // Inner
        }

        listener(found, Point(mp.get(0, 0)[0], mp.get(1, 0)[0]), Point(mp.get(2, 0)[0], mp.get(3, 0)[0]), Point(mp.get(4, 0)[0], mp.get(5, 0)[0]), max(tyMat.width(), tyMat.height()).toFloat(), m1, m2)
        //listener(found, Point(mmP.get(0, 0)[0], mmP.get(1, 0)[0]), Point(mmP.get(2, 0)[0], mmP.get(3, 0)[0]), max(tyMat.width(), tyMat.height()).toFloat())

        mKalmanFilter.correct(mmP)
        mmP.release()
        tyMat.release()
        mp.release()
        //yMat.release()
        //cMat.release()


        lMcorners.forEach { it.release() }
        imageProxy.close()
    }
}