#include "jvm.h"

#include "log.h"

std::string android_ros::GetPackageName(ANativeActivity* activity) {
  JNIEnv* env = nullptr;
  activity->vm->AttachCurrentThread(&env, nullptr);

  jclass android_content_Context = env->GetObjectClass(activity->clazz);
  jmethodID midGetPackageName = env->GetMethodID(
      android_content_Context, "getPackageName", "()Ljava/lang/String;");
  auto packageName =
      (jstring)env->CallObjectMethod(activity->clazz, midGetPackageName);

  return std::string(env->GetStringUTFChars(packageName, nullptr));
}

// https://stackoverflow.com/a/57656736
/**
 * \brief Gets the internal name for an android permission.
 * \param[in] lJNIEnv a pointer to the JNI environment
 * \param[in] perm_name the name of the permission, e.g.,
 *   "READ_EXTERNAL_STORAGE", "WRITE_EXTERNAL_STORAGE".
 * \return a jstring with the internal name of the permission,
 *   to be used with android Java functions
 *   Context.checkSelfPermission() or Activity.requestPermissions()
 */
jstring android_permission_name(JNIEnv* lJNIEnv, const char* perm_name) {
  // nested class permission in class android.Manifest,
  // hence android 'slash' Manifest 'dollar' permission
  jclass ClassManifestpermission =
      lJNIEnv->FindClass("android/Manifest$permission");
  jfieldID lid_PERM = lJNIEnv->GetStaticFieldID(
      ClassManifestpermission, perm_name, "Ljava/lang/String;");
  jstring ls_PERM = (jstring)(
      lJNIEnv->GetStaticObjectField(ClassManifestpermission, lid_PERM));
  return ls_PERM;
}

/**
 * \brief Tests whether a permission is granted.
 * \param[in] app a pointer to the android app.
 * \param[in] perm_name the name of the permission, e.g.,
 *   "READ_EXTERNAL_STORAGE", "WRITE_EXTERNAL_STORAGE".
 * \retval true if the permission is granted.
 * \retval false otherwise.
 * \note Requires Android API level 23 (Marshmallow, May 2015)
 */
bool android_ros::HasPermission(struct ANativeActivity* activity,
                                const char* perm_name) {
  JavaVM* lJavaVM = activity->vm;
  JNIEnv* lJNIEnv = nullptr;
  bool lThreadAttached = false;

  // Get JNIEnv from lJavaVM using GetEnv to test whether
  // thread is attached or not to the VM. If not, attach it
  // (and note that it will need to be detached at the end
  //  of the function).
  switch (lJavaVM->GetEnv((void**)&lJNIEnv, JNI_VERSION_1_6)) {
    case JNI_OK:
      break;
    case JNI_EDETACHED: {
      jint lResult = lJavaVM->AttachCurrentThread(&lJNIEnv, nullptr);
      if (lResult == JNI_ERR) {
        throw std::runtime_error("Could not attach current thread");
      }
      lThreadAttached = true;
    } break;
    case JNI_EVERSION:
      throw std::runtime_error("Invalid java version");
  }

  bool result = false;

  jstring ls_PERM = android_permission_name(lJNIEnv, perm_name);

  jint PERMISSION_GRANTED = jint(-1);
  {
    jclass ClassPackageManager =
        lJNIEnv->FindClass("android/content/pm/PackageManager");
    jfieldID lid_PERMISSION_GRANTED = lJNIEnv->GetStaticFieldID(
        ClassPackageManager, "PERMISSION_GRANTED", "I");
    PERMISSION_GRANTED =
        lJNIEnv->GetStaticIntField(ClassPackageManager, lid_PERMISSION_GRANTED);
  }
  {
    jclass ClassContext = lJNIEnv->FindClass("android/content/Context");
    jmethodID MethodcheckSelfPermission = lJNIEnv->GetMethodID(
        ClassContext, "checkSelfPermission", "(Ljava/lang/String;)I");
    jint int_result = lJNIEnv->CallIntMethod(
        activity->clazz, MethodcheckSelfPermission, ls_PERM);
    result = (int_result == PERMISSION_GRANTED);
  }

  if (lThreadAttached) {
    lJavaVM->DetachCurrentThread();
  }

  return result;
}

/**
 * \brief Query file permissions.
 * \details This opens the system dialog that lets the user
 *  grant (or deny) the permission.
 * \param[in] app a pointer to the android app.
 * \note Requires Android API level 23 (Marshmallow, May 2015)
 */
void android_ros::RequestPermission(ANativeActivity* activity,
                                    const char* permission) {
  LOGI("Requesting permission %s", permission);
  JavaVM* lJavaVM = activity->vm;
  JNIEnv* lJNIEnv = nullptr;
  bool lThreadAttached = false;

  // Get JNIEnv from lJavaVM using GetEnv to test whether
  // thread is attached or not to the VM. If not, attach it
  // (and note that it will need to be detached at the end
  //  of the function).
  switch (lJavaVM->GetEnv((void**)&lJNIEnv, JNI_VERSION_1_6)) {
    case JNI_OK:
      break;
    case JNI_EDETACHED: {
      jint lResult = lJavaVM->AttachCurrentThread(&lJNIEnv, nullptr);
      if (lResult == JNI_ERR) {
        throw std::runtime_error("Could not attach current thread");
      }
      lThreadAttached = true;
    } break;
    case JNI_EVERSION:
      throw std::runtime_error("Invalid java version");
  }

  LOGI("Seem to be attached?");

  jobjectArray perm_array = lJNIEnv->NewObjectArray(
      1, lJNIEnv->FindClass("java/lang/String"), lJNIEnv->NewStringUTF(""));

  LOGI("Created an array for permissions");

  lJNIEnv->SetObjectArrayElement(perm_array, 0,
                                 android_permission_name(lJNIEnv, permission));

  LOGI("Set the first permission on the array");

  jclass ClassActivity = lJNIEnv->FindClass("android/app/Activity");

  jmethodID MethodrequestPermissions = lJNIEnv->GetMethodID(
      ClassActivity, "requestPermissions", "([Ljava/lang/String;I)V");

  // TODO before requesting permissions use RegisterNatives to
  // make a callback for the result of the permissions request
  // https://stackoverflow.com/a/32919074
  LOGI("Got the requestPermissions method, about to call it");

  // Last arg (0) given as requestCode to onRequestPermissionsResult
  lJNIEnv->CallVoidMethod(activity->clazz, MethodrequestPermissions, perm_array,
                          0);
  if (lJNIEnv->ExceptionCheck()) {
    LOGI("An Exception occurred when requestion permissions");
  } else {
    LOGI("No Exception occurred when requestion permissions");
  }

  LOGI("About to detach thread");

  if (lThreadAttached) {
    lJavaVM->DetachCurrentThread();
  }
  LOGI("All done requesting permissions");
}
