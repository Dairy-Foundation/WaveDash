plugins {
	id("dev.frozenmilk.android-library") version "10.1.1-0.1.3"
	id("dev.frozenmilk.publish") version "0.0.4"
	id("dev.frozenmilk.doc") version "0.0.4"
}

android.namespace = "dev.frozenmilk.wavedash"

// Most FTC libraries will want the following
ftc {
	kotlin // if you don't want to use kotlin, remove this

	sdk {
		RobotCore
		FtcCommon {
			configurationNames += "testImplementation"
		}
	}
}

repositories {
	mavenCentral()
	maven("https://repo.dairy.foundation/releases/")
	maven("https://maven.brott.dev/")
}

dependencies {
	implementation("com.acmerobotics.roadrunner:ftc:0.1.14")
	implementation("com.acmerobotics.roadrunner:core:1.0.0")
	implementation("com.acmerobotics.roadrunner:actions:1.0.0")
	implementation("com.acmerobotics.dashboard:dashboard:0.4.16")

	implementation("dev.frozenmilk.dairy:Core:1.0.1")
	implementation("dev.frozenmilk.dairy:Pasteurized:1.0.0")
	implementation("dev.frozenmilk.mercurial:Mercurial:1.0.0")
	implementation("org.firstinspires.ftc:RobotCore:10.1.1")
}

publishing {
	publications {
		register<MavenPublication>("release") {
			groupId = "dev.frozenmilk"
			artifactId = "wavedash"

			artifact(dairyDoc.dokkaHtmlJar)
			artifact(dairyDoc.dokkaJavadocJar)

			afterEvaluate {
				from(components["release"])
			}
		}
	}
}
