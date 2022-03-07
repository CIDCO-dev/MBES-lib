pipeline {

  environment {
    major=0
    minor=1
    patch="${env.BUILD_ID}"
    name="${env.JOB_NAME}"
    version="$major.$minor.$patch"
    exec_name="datagram-dump-$version"
    publishDir="/var/www/html/$name/$version"
    publishDocDir="/var/www/html/$name/$version/doc"
    publishDoxygenDocDir="/var/www/html/$name/$version/doc/doxygen"
    publishCoberturaDir="/var/www/html/$name/$version/cobertura"
    lastPublishDir="/var/www/html/$name/last"
    binMasterPublishDir="$publishDir/Linux"
    binWinx64Dir="windows-x64"
    binWinx64PublishDir="$publishDir/$binWinx64Dir"
  }

  agent none
  stages {
  
    stage('COVERAGE'){
      agent { label 'ubnt20-build-opensidescan-vm'}
      steps {
        sh "make clean"
        sh "make coverage"
      }
      post {
        always {
          publishCppcheck pattern:'build/coverage/report/cppcheck.xml'
          step([$class: 'CoberturaPublisher', autoUpdateHealth: false, autoUpdateStability: false, coberturaReportFile: 'build/coverage/report/gcovr-report.xml', failUnhealthy: false, failUnstable: false, maxNumberOfBuilds: 0, onlyStable: false, sourceEncoding: 'ASCII', zoomCoverageChart: false])
          //sh 'mkdir -p $publishCoberturaDir'
          //sh 'cp -r build/coverage/report/*.html $publishCoberturaDir/'
          archiveArtifacts('build/coverage/report/*.html')
        }
      }
    }

    stage('DOCUMENTATION'){
      agent { label 'ubnt20-build-opensidescan-vm'}
      steps {
        sh "make doc"
      }
      post {
        always {
          //sh 'mkdir -p $publishDocDir'
          //sh 'mkdir -p $publishDoxygenDocDir'
          //sh 'cp -r build/doxygen/* $publishDoxygenDocDir/'
          sh 'zip -r build/doxygen/doxygen.zip build/doxygen '
          archiveArtifacts('build/doxygen/*.zip')
        }
      }
    }

    stage('BUILD WINDOWS 10 AND TEST'){
      agent { label 'windows10-x64-2'}
      steps {
	  
		bat "Scripts\\windowsBuildAndTest.bat"
		
        //bat "Scripts\\package_overlap.bat"

        archiveArtifacts('build\\bin\\datagram-dump.exe')
        archiveArtifacts('build\\bin\\cidco-decoder.exe')
        archiveArtifacts('build\\bin\\datagram-list.exe')
        archiveArtifacts('build\\bin\\georeference.exe')
		archiveArtifacts('build\\bin\\bounding-box.exe')
        archiveArtifacts('build\\bin\\data-cleaning.exe')
        archiveArtifacts('build\\bin\\raytrace.exe')
        archiveArtifacts('build\\bin\\datagram-raytracer.exe')

      }
      post {
        always {
          junit 'build\\reports\\*.xml'
        }
      }
    }

    stage('BUILD LINUX AND TEST'){
      agent { label 'ubnt20-build-opensidescan-vm'}
      steps {
        sh 'Scripts/linuxBuildAndTest.bash'
		
		archiveArtifacts('build/bin/datagram-dump')
		archiveArtifacts('build/bin/cidco-decoder')
		archiveArtifacts('build/bin/datagram-list')
		archiveArtifacts('build/bin/georeference')
		archiveArtifacts('build/bin/bounding-box')
		archiveArtifacts('build/bin/data-cleaning')
		archiveArtifacts('build/bin/raytrace')
		archiveArtifacts('build/bin/datagram-raytracer')
      }
      post {
        always {
          junit 'build/reports/*.xml'
        }
      }
    }
  }

}
