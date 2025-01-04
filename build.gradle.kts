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
		RobotCore("8.2.0-10.0.0-10.1.1")
		FtcCommon("8.2.0-10.1.1")
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
	implementation("com.acmerobotics.dashboard:dashboard:0.4.16")

	implementation("dev.frozenmilk.dairy:Core:1.0.1")
	implementation("dev.frozenmilk.dairy:Pasteurized:1.0.0")
	implementation("dev.frozenmilk.mercurial:Mercurial:1.0.0")
}

publishing {
	publications {
		register<MavenPublication>("releases") {
			groupId = "dev.frozenMilk"
			artifactId = "Wavedash"

			artifact(dairyDoc.dokkaHtmlJar)
			artifact(dairyDoc.dokkaJavadocJar)

			afterEvaluate {
				from(components["release"])
			}
		}

		register<MavenPublication>("snapshots") {
			groupId = "dev.frozenMilk"
			artifactId = "Wavedash"

			artifact(dairyDoc.dokkaHtmlJar)
			artifact(dairyDoc.dokkaJavadocJar)

			afterEvaluate {
				from(components["release"])
			}
		}
	}
}
