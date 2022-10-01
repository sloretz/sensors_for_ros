# ROS for Android

ROS for Android is an app that publishes sensor data from an Android device onto ROS 2 topics.
Currently it supports ROS Humble.

**Supported sensors**
* Accelerometer
* Barometer
* Camera(s)
* Gyroscope
* Illuminance
* Magnetometer

This app is built using only CMake and C++.
It does not use Java or Kotlin.
ROS 2 packages up to `rclcpp` are cross-compiled.
A successful build produces an `.apk` file called `android_ros.apk` in the build directory.

## How to install it

Currently the only way to get **ROS for Android** is to build it from source.
It is not yet available on Google's app store.

## How to build it from source

You do not need ROS installed on your machine to build the **ROS for Android** app.
However, it's needed to use the sensor data being published by your Android device.
Follow [these instructions to install ROS Humble](https://docs.ros.org/en/humble/Installation.html).

### Computer setup

Download the [Android SDK "Command-line tools only" version](https://developer.android.com/studio#command-tools).
Other versions may work, but this is the minimum needed.

Make a folder for the SDK and extract the archive.

```bash
mkdir ~/android-sdk
cd ~/android-sdk
unzip ~/Downloads/commandlinetools-linux-8512546_latest.zip
```

Install some Android SDK components

```
./cmdline-tools/bin/sdkmanager --sdk_root=$HOME/android-sdk "build-tools;33.0.0" "platforms;android-30" "ndk;25.1.8937393"
```

Install `adb`

```bash
sudo apt install adb android-sdk-platform-tools-common
```

You may need to add yourself to the `plugdev` group.
Follow the [Set up a device for development](https://developer.android.com/studio/run/device#setting-up) instructions.


### Create debug keys

You'll need to install openjdk to get access to `keytool`.

```bash
sudo apt install openjdk-11-jre-headless
```

Create a debug keystore

```bash
mkdir ~/.android
keytool -genkey -v -keystore ~/.android/debug.keystore -alias adb_debug_key -keyalg RSA -keysize 2048 -validity 10000 -storepass android -keypass android
```

### Clone the repo

The official repo is [`sloretz/android_ros`](https://github.com/sloretz/android_ros.git).

```
git clone https://github.com/sloretz/android_ros.git
```

### Download dependencies

TODO vcstool to download the repos files for native and cross-compiled dependencies

### Building the App

Build the software

```
mkdir build
cd build
cmake ../ -DANDROID_HOME=$HOME/android-sdk/
make -j`nproc`
```

### Installing the App on your Android Device

Install the APK in the build directory onto a device.

```
adb install -r android_ros.apk
```

## Development tips

Use logcat to view the logs from the app
```
adb logcat
```

Sometimes you may want to try out a permission without writing the code to request it.
The app must be installed, but not running already for this command to work.
```
adb shell pm grant loretz.shane android.permission.CAMERA
```

The main activity can be started directly from the CLI
```
adb shell am start -n loretz.shane/android.app.NativeActivity
```

Getting stack traces

```
adb logcat | $HOME/android-sdk/ndk/*/ndk-stack -sym lib/arm64-v8a/
```

# Random lessons

During development I documented problems I encountered and fixes for them in the [Problems Encountered](doc/problems_encountered.md) document.
