pipeline {

    environment {
	major=0
	minor=1
	patch="${env.BUILD_ID}"
        name="${env.JOB_NAME}"
        version="$major.$minor.$patch"
	exec_name="datagram-dump-$version"
        publishDir="/var/www/html/$name/$version"
        lastPublishDir="/var/www/html/$name/last"
	binMasterPublishDir="$publishDir/Linux"
	binWinx64Dir="windows-x64"
	binWinx64PublishDir="$publishDir/$binWinx64Dir"
    }

    agent none
    stages {

        stage('STAR VM'){
    	    agent { label 'master'}
            steps {
              sh 'ssh hugo@192.168.0.219 "bash -s" < /var/lib/jenkins/Scripts/Start_A_VM.sh windows-x64-C++'
            }
        }	

        stage('TEST MASTER'){
            agent { label 'master'}
            steps {
              sh "make test"
            }
        }

        stage('DOCUMENTATION'){
          agent { label 'master'}
          steps {
              sh "make doc"
            }
        }

        stage('BUILD WINDOWS 10 AND TEST'){
          agent { label 'windows-x64-2'}
          steps {
		bat "Scripts\\change_makefile_name.bat"
      		//compile
		bat "make test"
		archiveArtifacts('build\\bin\\datagram-dump.exe')
            }
	  post {
	     always {
		junit 'build\\test-reports\\*.xml'
	     }
	     failure{
      		   timeout(time: 10, unit: 'SECONDS'){
		     bat "ssh jenkins@192.168.0.105 /var/lib/jenkins/Scripts/Call_Close_A_VM.sh windows-x64-C++"
		   }
	     }
	     aborted{
      		   timeout(time: 10, unit: 'SECONDS'){
		     bat "ssh jenkins@192.168.0.105 /var/lib/jenkins/Scripts/Call_Close_A_VM.sh windows-x64-C++"
		   }
	     }
	  }
        }

        stage('STOP VM'){
    	    agent { label 'master'}
            steps {
              sh 'ssh hugo@192.168.0.219 "bash -s" < /var/lib/jenkins/Scripts/Close_A_VM.sh windows-x64-C++'
            }
        }	

        stage('BUILD MASTER'){
            agent { label 'master'}
            steps {
              sh 'make'
              sh 'mkdir -p $binMasterPublishDir'
              sh 'cp -r build/bin/datagram-dump $binMasterPublishDir/$exec_name'
            }
        }

        stage('PUBLISH ON SERVER'){
            agent { label 'master'}
            steps {
              sh 'mkdir -p $binWinx64PublishDir'
              sh 'cp  /var/lib/jenkins/jobs/$name/builds/$patch/archive/build/bin/windows-x64/datagram-dump.exe  $binWinx64PublishDir/$exec_name.exe'
            }
        }
    }

}
