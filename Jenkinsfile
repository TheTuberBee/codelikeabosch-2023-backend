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
                echo 'docker run -d -p 8000:8000 -e DB_HOST={DB_HOST} -e DB_PORT={DB_PORT} -e DB_USER={DB_USER} -e DB_PASSWORD={DB_PASSWORD} -e DB_DATABASE={DB_DATABASE} bosch'
                sh 'docker run -d -p 8000:8000 -e DB_HOST={DB_HOST} -e DB_PORT={DB_PORT} -e DB_USER={DB_USER} -e DB_PASSWORD={DB_PASSWORD} -e DB_DATABASE={DB_DATABASE} bosch'
            }
        }
    }
}
