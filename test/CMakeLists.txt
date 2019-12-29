set(INSTALL_GTEST OFF CACHE BOOL "Install google-test" FORCE)

# ----- Download google-test -----
# Download projectというCMake projectでGoogleTestのソースコードを自動ダウンロードして利用するようにする
# GoogleTestのバージョン違い環境準備が必要なくなるのが利点？
# ネイティブビルドできるのでXavier直ビルドするときに便利かもしれない
# include(${PROJECT_SOURCE_DIR}/cmake/DownloadProject.cmake)
# download_project(PROJ googletest
#                  GIT_REPOSITORY https://github.com/google/googletest.git
#                  GIT_TAG master
#                  UPDATE_DISCONNECTED 1)
# add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})

# ビルド済みのバイナリを利用する場合
# バージョン、アーキテクチャ固定になるので注意
# だがオフラインでも利用できるので便利
set (GTEST_ROOT ${CMAKE_SOURCE_DIR}/3rd/gtest)
find_package(GTest REQUIRED)
include(GoogleTest)


# ----- Glob test codes----- 
# テストしたいコードをまとめて追加するようにしている、ここでは<PROJECT_ROOT/test/sample>以下のcppを追加するようにしている。
file(GLOB_RECURSE TEST_SAMPLE_SOURCE_PATHS "./sample/*.cpp")
list(APPEND SOURCE_PATHS ${TEST_SAMPLE_SOURCE_PATHS})

# ----- Build test executables -----
# ${SOURCE_PATHS}にテストコードをすべて入れておく。これに対して次の処理を加えている
# 1. TEST_MODULCE_NAMEを生成する：おそらくMolude nameは"sample/sample1.cpp"のテストコードから、"sample_sample1"にきまるぽい
# 2. add_executableする。シンボル名はTEST_MODULDE_NAME。このときTestに利用するサンプルデータのパスをTEST_DATA_DIRとしてDefinitionに与えるようになっている
# 3. Include dirの設定。Project本体のインクルードパスとGoogleTestのInclude pathが設定されている
# 4. Linkの設定。必要なライブラリに加えてGoogleTest関係とみられるgtest_mainも追加されている
foreach(SOURCE_PATH ${SOURCE_PATHS})
    # Get relative path from ./test/
    file(RELATIVE_PATH SOURCE_REL_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${SOURCE_PATH})
    # Test module name: test_foo_bar
    string(REGEX REPLACE "\\.cpp$" "" TEST_MODULE_NAME test/${SOURCE_REL_PATH})
    string(REPLACE "." "_" TEST_MODULE_NAME ${TEST_MODULE_NAME})
    string(REPLACE "/" "_" TEST_MODULE_NAME ${TEST_MODULE_NAME})
    # Executable name: test_foo_bar
    set(TEST_EXECUTABLE_NAME ${TEST_MODULE_NAME})

    # Create test executable
    add_executable(${TEST_EXECUTABLE_NAME} ${SOURCE_PATH})
    target_compile_definitions(${TEST_EXECUTABLE_NAME} PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data/")
    target_include_directories(${TEST_EXECUTABLE_NAME} SYSTEM
                               PRIVATE
                               ${PROJECT_SOURCE_DIR}/src
                               # ${googletest_SOURCE_DIR}/googletest/include
                               ${GTEST_INCLUDE_DIRS}
                               )
    # target_link_libraries(${TEST_EXECUTABLE_NAME}
    #                       PRIVATE
    #                       ${PROJECT_NAME}
    #                       gtest_main
    #                       opencv_imgcodecs
    #                       opencv_highgui)
    target_link_libraries(${TEST_EXECUTABLE_NAME}
                          PRIVATE
                          ${PROJECT_NAME}
                          GTest::GTest
                          GTest::Main
                          opencv_imgcodecs
                          opencv_highgui)
    
    set_target_properties(${TEST_EXECUTABLE_NAME} PROPERTIES
                          RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/test
                          RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/test
                          RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL ${PROJECT_BINARY_DIR}/test
                          RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${PROJECT_BINARY_DIR}/test)

    # Add test
    # テストの実行ファイルを追加する。ここで追加したテストはctestか、make testコマンドで自動実行されるようになる。
    # cmakeの機能であるadd_testで登録ができる。しかし、テストの実行をしてもどのテストケースが成功したか？などよくわからないという問題がある。
    # そこで、gtestの機能であるgtest_add_testを利用することで一つ一つのテストケースの成否が出力されるようになる。
    # 実行バイナリ一つの登録であるadd_testを、実行バイナリ内の複数テストケース分実装するようにしたのが、gtest_add_test?
    # しかしながら、本CMakeLists.txtではなぜか、gtest_add_testがうまくいかない（cmake ..怒られてしまう。）
    # かわりにgtest_discover_testsでtraget symbolを登録するとうまく行ったので、これを利用するようにする。
    

    gtest_discover_tests(${TEST_EXECUTABLE_NAME}) # << テストバイナリ内での全テストケースについて成否を出力できるので便利。
    # gtest_add_tests(TARGET ${TEST_EXECUTABLE_NAME}) # << うまくいかない。何が行けないかは不明なので要調査。
    # add_test(${TEST_MODULE_NAME} ${PROJECT_BINARY_DIR}/test/${TEST_EXECUTABLE_NAME}) # << うまく行くが、登録したテストバイナリ全体での成否しかわからずちょっと不便。

endforeach()
