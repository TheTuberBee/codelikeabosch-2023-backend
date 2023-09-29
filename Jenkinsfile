pipeline {
    agent any
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
                sh 'docker run -d -p 8000:8000 bosch' \
                -e DB_HOST={DB_HOST} \
                -e DB_PORT={DB_PORT} \
                -e DB_USER={DB_USER} \
                -e DB_PASSWORD={DB_PASSWORD} \
                -e DB_DATABASE={DB_DATABASE}
            }
        }
    }
}
