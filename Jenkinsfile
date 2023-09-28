pipeline {
    agent none
    stages {
        stage('Stop') {
            steps {
                sh 'docker stop bosch || true'
            }
        }
        stage('Build') {
            steps {
                sh 'docker build -t bosch .'
            }
        }
        stage('Run') {
            steps {
                sh 'docker run -d -p 8000:8000 bosch'
            }
        }
    }
}