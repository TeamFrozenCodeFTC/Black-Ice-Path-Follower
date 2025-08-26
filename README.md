# FTC Black Ice Path Follower

Please visit our website and documentation: https://teamfrozencodeftc.github.io/Black-Ice-Path-Follower/

Jump to the code powering Black Ice: https://github.com/TeamFrozenCodeFTC/Black-Ice-Path-Follower/tree/main/TeamCode/src/main/java/org/firstinspires/ftc/blackice

## Requirements
1. Program in Java and use Android Studio. Steps to setup Android Studio [here](https://ftc-docs.firstinspires.org/en/latest/programming_resources/tutorial_specific/android_studio/installing_android_studio/Installing-Android-Studio.html) if you haven't already. 
2. Have odometry wheels or some form of localization.

# Getting Started

If you already have a ftc reposition, [install Black Ice into that existing project](#installing-into-an-existing-project). 

Otherwise you can create a new project with Black Ice pre-installed [here](#creating-a-new-project).


## Creating a New Project
Decide a folder location, open a terminal, and enter this command. Then open the project in android studio.
```bash
git clone https://github.com/TeamFrozenCodeFTC/Black-Ice-Path-Follower.git
```

## Installing into an Existing Project

1. Down the Black Ice and put the folder inside your repository [here]( https://downgit.evecalm.com/#/home?url=https://github.com/TeamFrozenCodeFTC/Black-Ice-Path-Follower/tree/main/TeamCode/src/main/java/org/firstinspires/ftc/blackice).
2. Go to `build.dependencies.gradle` and add the following line to the **repositories** block:
```gradle
maven { url = 'https://maven.brott.dev/' }
```
Then add this line to the **dependencies** block:
```gradle
implementation 'com.acmerobotics.dashboard:dashboard:0.4.17'
```
It should look something like this:
```gradle
repositories {
    mavenCentral()
    google() // Needed for androidx
    maven { url = 'https://maven.brott.dev/' }
}

dependencies {
    implementation 'org.firstinspires.ftc:Inspection:10.3.0'
    implementation 'org.firstinspires.ftc:Blocks:10.3.0'
    implementation 'org.firstinspires.ftc:RobotCore:10.3.0'
    implementation 'org.firstinspires.ftc:RobotServer:10.3.0'
    implementation 'org.firstinspires.ftc:OnBotJava:10.3.0'
    implementation 'org.firstinspires.ftc:Hardware:10.3.0'
    implementation 'org.firstinspires.ftc:FtcCommon:10.3.0'
    implementation 'org.firstinspires.ftc:Vision:10.3.0'
    implementation 'androidx.appcompat:appcompat:1.2.0'
    implementation 'com.acmerobotics.dashboard:dashboard:0.4.17'
}
```
