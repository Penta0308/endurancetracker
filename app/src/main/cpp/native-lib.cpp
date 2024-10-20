#include <jni.h>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/flann.hpp>
#include <opencv2/features2d.hpp>

static cv::ORB* orb;

extern "C" JNIEXPORT void JNICALL
Java_kr_glora_endurancetracker_MainActivity_detInit(
        JNIEnv* env,
        jobject /* this */) {
    orb = cv::ORB::create();
}

extern "C" JNIEXPORT jdoubleArray JNICALL
Java_kr_glora_endurancetracker_MainActivity_detPos(JNIEnv* env, jobject/* this*/, jlong matNativeObjAddr) {
    //orb->detectAndCompute(reinterpret_cast<cv::Mat*>(matNativeObjAddr)->rows, cv::noArray(), std::vector<>, nullptr);
    return env->NewDoubleArray(2);
}