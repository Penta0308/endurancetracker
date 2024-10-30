package kr.glora.endurancetracker

import org.opencv.core.Mat
import kotlin.math.max

fun mapToCenter(p: DoubleArray, mat: Mat): DoubleArray {
    val scalew = max(mat.width(), mat.height())
    return doubleArrayOf((p[0] - mat.width() / 2) / scalew, (p[1] - mat.height() / 2) / scalew)
}