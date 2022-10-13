# Sensors for ROS

Sensors for ROS is an app that publishes sensor data from an Android device onto ROS 2 topics.
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
A successful build produces an `.apk` file called `sensors_for_ros.apk` in the build directory.

## Inspiration

These projects were extremely helpful, and used as a reference for this one:

* https://github.com/cnlohr/rawdrawandroid
* https://github.com/ocornut/imgui/tree/master/examples/example_android_opengl3
* https://www.sisik.eu/blog/android/ndk/camera

## How to install it

Currently the only way to get **Sensors for ROS** is to build it from source.
It is not yet available on Google's app store.

## How to build it from source

You do not need ROS installed on your machine to build the **Sensors for ROS** app.
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
# If you're using Ubuntu
sudo apt install adb android-sdk-platform-tools-common
# If you're using Fedora
sudo dnf install android-tools
```

Install catkin-pkg, empy, and lark

```bash
# If you're using ubuntu
sudo apt install python3-catkin-pkg-modules python3-empy python3-lark-parser
# If you're using Fedora
sudo dnf install python3-catkin_pkg python3-empy python3-lark-parser
```

You may need to do additional setup to use adb.
Follow the [Set up a device for development](https://developer.android.com/studio/run/device#setting-up) instructions if you're using Ubuntu, or follow [the instructions in this thread](https://forums.fedoraforum.org/showthread.php?298965-HowTo-set-up-adb-(Android-Debug-Bridge)-on-Fedora-20) if you're using Fedora.


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

The official repo is [`sloretz/sensors_for_ros`](https://github.com/sloretz/sensors_for_ros).

```
git clone https://github.com/sloretz/sensors_for_ros.git
```

Next initialize the git submodules.

```bash
git submodule init
git submodule update
```

### Download ROS dependencies

Use [vcstool](https://github.com/dirk-thomas/vcstool) to download the ROS packages we need to cross compile into the `deps` folder.

```
vcs import --input ros.repos deps/
```

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
adb install -r sensors_for_ros.apk
```

## Development tips

Use logcat to view the logs from the app
```
adb logcat
```

Sometimes you may want to try out a permission without writing the code to request it.
The app must be installed, but not running already for this command to work.
```
adb shell pm grant com.github.sloretz.sensors_for_ros android.permission.CAMERA
```

The main activity can be started directly from the CLI
```
adb shell am start -n com.github.sloretz.sensors_for_ros/android.app.NativeActivity
```

Getting stack traces

```
adb logcat | $HOME/android-sdk/ndk/*/ndk-stack -sym lib/arm64-v8a/
```

Creeating a release build for the play store is currently a manuall process as google requires App bundles instead of apks.
See [Notes about using bundletool](https://developer.android.com/studio/build/building-cmdline)

```
ANDROID_KEY_PASS=??? ANDROID_KEYSTORE_PASS=??? cmake .. -DDEBUGGABLE=OFF -DPATH_TO_KEYSTORE=~/.android/release.keystore -DANDROID_HOME=$HOME/projects/android-sdk
```

# Random lessons

During development I documented problems I encountered and fixes for them in the [Problems Encountered](docs/problems_encountered.md) document.


# Creating a release for the play store

This is currently a manual process.
Creating a bundle for the Play Store is different from creating an installable APK, and is not yet automated.

```
cmake .. -DDEBUGGABLE=OFF -DANDROID_HOME=$HOME/projects/android-sdk
# Manually change ABI in CMakeLists.txt for all 4 android ABIs and make each one
make -j`nproc`
```

```
aapt2 link --proto-format -o base.zip -I ~/projects/android-sdk/platforms/android-30/android.jar --manifest AndroidManifest.xml -R compiled_resources/*.flat --auto-add-overlay
```

Extract `base.zip` and make a file directory matching the bundle file directory.
Copy libs from each ABI into it.

```
java -jar ~/Downloads/bundletool-all-1.11.2.jar build-bundle --modules=basewithlibs.zip --output sensors_for_ros.aab
```

```
jarsigner -keystore /home/sloretz/.android/release.keystore -storepass ??? sensors_for_ros.aab adb_release_key
```
