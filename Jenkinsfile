pipeline {

    agent none
    stages {
        stage('TEST MASTER'){
            agent { label 'master'}
            steps {
              sh "make run-test"
            }
        }

        stage('DOCUMENTATION'){
          agent { label 'master'}
          steps {
              sh "make doc"
            }
        }
        stage('BUILD MASTER'){
            agent { label 'master'}
            steps {
              sh "make deploy"
            }
        }
    }

}
