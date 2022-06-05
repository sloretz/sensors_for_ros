#include "jvm.h"

std::string android_ros::GetPackageName(ANativeActivity* activity)
{
    JNIEnv* env = nullptr;
    activity->vm->AttachCurrentThread(&env, nullptr);

    jclass android_content_Context = env->GetObjectClass(activity->clazz);
    jmethodID midGetPackageName = env->GetMethodID(android_content_Context,
                                                   "getPackageName",
                                                   "()Ljava/lang/String;");
    auto packageName = (jstring)env->CallObjectMethod(activity->clazz,
                                                        midGetPackageName);

    return std::string(env->GetStringUTFChars(packageName, nullptr));
}
