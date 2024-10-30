package kr.glora.endurancetracker

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.util.AttributeSet
import android.view.View
import org.opencv.core.Point
import kotlin.math.max

class TrackOverlayView(context: Context, attrs: AttributeSet? = null) : View(context, attrs) {
    private var enduPos: Point? = null
    private var enduDir: Point? = null
    private var enduVel: Point? = null
    private var found: Boolean = false
    private var scalef: Float = 1.0f

    companion object {
        const val FPS = 60.0
    }

    private val paint: Paint = Paint().apply {
        color = Color.GREEN
        style = Paint.Style.STROKE
        strokeWidth = 10f
    }

    fun updateEndu(found: Boolean, enduPos: Point, enduDir: Point, enduVel: Point, scalef: Float) {
        this.found = found
        this.enduPos = enduPos
        this.enduDir = enduDir
        this.enduVel = enduVel
        //this.scalef = scalef
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)

        if (enduPos != null) {
            scalef = max(width, height).toFloat()
            canvas.drawCircle((-enduPos!!.y * scalef + width / 2).toFloat(),
                (enduPos!!.x * scalef + height / 2).toFloat(), 10f, paint)
            canvas.drawLine(
                (-enduPos!!.y * scalef + width / 2).toFloat(),
                (enduPos!!.x * scalef + height / 2).toFloat(),
                (-enduPos!!.y * scalef + width / 2 + -enduDir!!.y * scalef / 2).toFloat(),
                (enduPos!!.x * scalef + height / 2 + enduDir!!.x * scalef / 2).toFloat(),
                paint
            )
            canvas.drawLine(
                (-enduPos!!.y * scalef + width / 2).toFloat(),
                (enduPos!!.x * scalef + height / 2).toFloat(),
                (-enduPos!!.y * scalef + width / 2 + -enduVel!!.y * scalef * FPS).toFloat(),
                (enduPos!!.x * scalef + height / 2 + enduVel!!.x * scalef * FPS).toFloat(),
                paint
            )
        }
    }

}