#!/bin/bash
set -e

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd $SCRIPT_DIR

if [ -z "$ANDROID_HOME" ];
then
  echo "ANDROID_HOME environment vairable must be set" >&2
  exit
fi

ANDROID_ABI=arm64-v8a

rm -rf build/* install/*
mkdir -p build install

# Helpful stuff:
# https://developer.android.com/studio/command-line/aapt2
# https://developer.android.com/studio/command-line/bundletool
# https://developer.android.com/studio/build/building-cmdline
# https://developer.android.com/studio/build/building-cmdline#build_bundle
# https://developer.android.com/ndk/samples/sample_na
# https://developer.android.com/ndk/guides/cmake
# https://developer.android.com/reference/android/app/NativeActivity
# https://github.com/android/ndk-samples/blob/master/native-activity/app/src/main/cpp/CMakeLists.txt
# https://android.googlesource.com/platform/development/+/4948c163663ecc343c97e4c2a2139234f1d3273f/ndk/sources/android/native_app_glue/android_native_app_glue.h

cd build
cmake -DCMAKE_TOOLCHAIN_FILE=$ANDROID_HOME/ndk/21.3.6528147/build/cmake/android.toolchain.cmake \
    -DANDROID_ABI=$ANDROID_ABI \
    -DANDROID_PLATFORM=android-30 \
    -DCMAKE_INSTALL_PREFIX=../install \
    ../hello_android

cmake --build .

cd $SCRIPT_DIR

APPT2=/home/sloretz/android-sdk/build-tools/30.0.2/aapt2

# One invocation per resource (for incremental builds)
# $APPT2 compile \
#     TODO resource here, like a strings file or image
#   -o install

# $AAPT2 link --proto-format -o install/hello_android.apk -I $ANDROID_HOME/platforms/android_version/android.jar --manifest build/AndroidManifest.xml --auto-add-overlay


cd build
make install
cd -

# Compile resources using aapt2
mkdir build/compiled_resources

~/android-sdk/build-tools/30.0.2/aapt2 compile res/values/strings.xml -o build/compiled_resources
~/android-sdk/build-tools/30.0.2/aapt2 compile res/mipmap-hdpi/ic_launcher.png -o build/compiled_resources
~/android-sdk/build-tools/30.0.2/aapt2 compile res/mipmap-mdpi/ic_launcher.png -o build/compiled_resources
~/android-sdk/build-tools/30.0.2/aapt2 compile res/mipmap-xhdpi/ic_launcher.png -o build/compiled_resources
~/android-sdk/build-tools/30.0.2/aapt2 compile res/mipmap-xxhdpi/ic_launcher.png -o build/compiled_resources

~/android-sdk/build-tools/30.0.2/aapt2 link \
  -o install/hello_android-nolibs.apk \
  -I /home/sloretz/android-sdk/platforms/android-30/android.jar \
  --manifest build/AndroidManifest.xml \
  -R build/compiled_resources/*.flat \
  --auto-add-overlay

# TODO include this when using bundletool
  # --proto-format \

# How does aapt2 add ndk resources? No idea, zip it in
unzip install/hello_android-nolibs.apk -d install/hello_android-nolibs.apk-extracted
cp -R install/lib install/hello_android-nolibs.apk-extracted/
cd install/hello_android-nolibs.apk-extracted/
zip -D0r ../hello_android-unaligned.apk *
cd -

# align to 4KiB page boundary
~/android-sdk/build-tools/30.0.2/zipalign -p -f -v 4 \
  install/hello_android-unaligned.apk \
  install/hello_android.apk \

# Make sure it worked
~/android-sdk/build-tools/30.0.2/zipalign -c -v 4 install/hello_android.apk

echo "Signing the APK"

# Sign the APK
export ANDROID_KEY_PASS=android
export ANDROID_KEYSTORE_PASS=android
~/android-sdk/build-tools/30.0.2/apksigner sign \
  --ks ~/.android/debug.keystore \
  --ks-pass env:ANDROID_KEYSTORE_PASS \
  --key-pass env:ANDROID_KEY_PASS \
  install/hello_android.apk

echo "Verifying the APK signature"

~/android-sdk/build-tools/30.0.2/apksigner verify \
  install/hello_android.apk

# Already ran this to create debug keystore
# keytool -genkey -v -keystore ~/.android/debug.keystore -keyalg RSA -keysize 2048 -validity 10000 -storepass android -keypass android -dname "CN=example.com, OU=ID, O=Example, L=Doe, S=John, C=GB"
