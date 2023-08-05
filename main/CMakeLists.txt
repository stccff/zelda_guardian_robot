#自动查找头文件路径函数(增加去重部分)
macro(FIND_INCLUDE_DIR result curdir)  #定义函数,2个参数:存放结果result；指定路径curdir；
    file(GLOB_RECURSE children "${curdir}/*.hpp" "${curdir}/*.h" )	#遍历获取{curdir}中*.hpp和*.h文件列表
    message(STATUS "children= ${children}")								#打印*.hpp和*.h的文件列表
    set(dirlist "")														#定义dirlist中间变量，并初始化
    foreach(child ${children})											#for循环
        string(REGEX REPLACE "(.*)/.*" "\\1" LIB_NAME ${child})			#字符串替换,用/前的字符替换/*h
        if(IS_DIRECTORY ${LIB_NAME})									#判断是否为路径
            if(dirlist MATCHES ${LIB_NAME})#判断dirlist是否含有${LIB_NAME}
            else()
              MESSAGE(STATUS "current platform: Linux ")
              LIST(APPEND dirlist ${LIB_NAME})							#将合法的路径加入dirlist变量中  
            endif()
        endif()															#结束判断
    endforeach()														#结束for循环
    set(${result} ${dirlist})											#dirlist结果放入result变量中
endmacro()

#添加源文件
file(GLOB_RECURSE srcs *.c)

#查找include目录下的所有*.hpp,*.h头文件,并路径列表保存到 INCLUDE_DIR_LIST 变量中
FIND_INCLUDE_DIR(include_dirs ./)			#调用函数，指定参数

# set(srcs "main.c"
# )
# set(include_dirs "."
# )

# set(embed_files "sound/sight_search.pcm")

idf_component_register(SRCS "${srcs}"
                       EMBED_FILES "${embed_files}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES esp_hid
                       REQUIRES driver
                       REQUIRES esp_adc
                       REQUIRES spiffs
                       REQUIRES mbedtls
                       PRIV_REQUIRES nvs_flash)
                       
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")


# Create a SPIFFS image from the contents of the 'spiffs_image' directory
# that fits the partition named 'storage'. FLASH_IN_PROJECT indicates that
# the generated image should be flashed when the entire project is flashed to
# the target with 'idf.py -p PORT flash'.
# spiffs_create_partition_image(storage ../spiffs_image FLASH_IN_PROJECT)