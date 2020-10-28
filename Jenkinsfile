pipeline {
    agent any

    options {
        buildDiscarder(logRotator(numToKeepStr: '5', artifactNumToKeepStr: '3'))
    }

    stages {
        stage('Checkout (Common)') {
            steps {
                checkout([
                    $class: 'GitSCM',
                    branches: [
                        [name: "*/${env.BRANCH_NAME}"]
                    ],
                    browser: [
                        $class: 'GithubWeb',
                        repoUrl: 'https://github.com/PhischDotOrg/stm32-common'
                    ],
                    doGenerateSubmoduleConfigurations: false,
                    extensions: [
                        [ $class: 'SubmoduleOption',
                            depth: 1,
                            disableSubmodules: false,
                            parentCredentials: true,
                            recursiveSubmodules: true,
                            reference: '',
                            shallow: true,
                            threads: 8,
                            trackingSubmodules: false
                        ],
                        [ $class: 'RelativeTargetDirectory',
                            relativeTargetDir: 'stm32-common'
                        ],
                        [ $class: 'CleanCheckout' ]
                    ],
                    submoduleCfg: [
                    ],
                    userRemoteConfigs: [
                        [
                            credentialsId: 'a88f9971-dae1-4a4d-9a8f-c7af88cad71b',
                            url: 'git@github.com:PhischDotOrg/stm32-common.git'
                        ]
                    ]
                ])
            }
        }
        stage('Regression') {
            matrix {
                axes {
                    axis {
                        name 'STM32_PROJECT'
                        values 'stm32f4-minimal',
                          'stm32f1-bluepill',
                          'stm32f4-nucleo64',
                          'stm32f4-usbdevice'
                        //   'stm32l4-nucleo32'
                    }
                }
                stages {

                    stage('Checkout (Projects)') {
                        steps {
                            checkout([ $class: 'GitSCM',
                                branches: [ [name: '*/stm32-common']],
                                browser: [ $class: 'GithubWeb',
                                    repoUrl: "https://github.com/PhischDotOrg/${STM32_PROJECT}"
                                ],
                                doGenerateSubmoduleConfigurations: false,
                                extensions: [
                                    [ $class: 'RelativeTargetDirectory', relativeTargetDir: "${STM32_PROJECT}" ]
                                ],
                                submoduleCfg: [],
                                userRemoteConfigs: [
                                    [ credentialsId: 'a88f9971-dae1-4a4d-9a8f-c7af88cad71b',
                                    url: "git@github.com:PhischDotOrg/${STM32_PROJECT}"
                                    ]
                                ]
                            ])

                            dir("${STM32_PROJECT}") {
                                sh 'pwd'
                                sh 'rm -rd common'
                                sh 'ln -s ../stm32-common common'
                            }
                        }
                    }
                    stage('Build (Unit Tests)') {
                        steps {
                            cmakeBuild buildDir: "${WORKSPACE}/${STM32_PROJECT}/build",
                                buildType: "Debug",
                                cleanBuild: true,
                                cmakeArgs: "-DUNITTEST=TRUE",
                                installation: 'InSearchPath',
                                sourceDir: "${WORKSPACE}/${STM32_PROJECT}",
                                steps: [
                                    [ args: 'all' ]
                                ]
                        }
                    }
                    stage('Test') {
                        steps {
                            ctest arguments: "-T test --no-compress-output",
                                installation: 'InSearchPath',
                                workingDir: "${WORKSPACE}/${STM32_PROJECT}/build"
                        }
                    }

                    stage('Collect Results') {
                        steps {
                            // Archive the CTest xml output
                            archiveArtifacts (
                                artifacts: "${STM32_PROJECT}/build/**/*.xml",
                                fingerprint: true
                            )

                            // Process the CTest xml output with the xUnit plugin
                            xunit (
                                testTimeMargin: '3000',
                                thresholdMode: 1,
                                thresholds: [
                                skipped(failureThreshold: '0'),
                                failed(failureThreshold: '0')
                                ],
                            tools: [CTest(
                                pattern: "${STM32_PROJECT}/build/**/*.xml",
                                deleteOutputFiles: true,
                                failIfNotNew: false,
                                skipNoTestFiles: true,
                                stopProcessingIfError: true
                                )]
                            )
                        }
                    }
                }
            }
        }
    }
}
