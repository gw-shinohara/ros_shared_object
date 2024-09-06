#include <dlfcn.h>
#include <iostream>

typedef RosSharedObjectLibrary* (*create_node_t)();
typedef void (*destroy_node_t)(RosSharedObjectLibrary*);

int main() {
    // 共有ライブラリをロード
    void* handle = dlopen("libros_shared_object_library.so", RTLD_LAZY);
    if (!handle) {
        std::cerr << "Failed to load library: " << dlerror() << std::endl;
        return 1;
    }

    // ノードを作成する関数と削除する関数を取得
    create_node_t create_node = (create_node_t)dlsym(handle, "create_node");
    destroy_node_t destroy_node = (destroy_node_t)dlsym(handle, "destroy_node");
    if (!create_node || !destroy_node) {
        std::cerr << "Failed to load symbols: " << dlerror() << std::endl;
        dlclose(handle);
        return 1;
    }

    // ノードを作成
    RosSharedObjectLibrary* node = create_node();
    
    // メッセージをパブリッシュ
    node->publish_message("Hello, ROS2 from shared object!");

    // ノードを開始
    node->start();

    // ノードを停止
    node->stop();

    // ノードを削除
    destroy_node(node);

    dlclose(handle);
    return 0;
}
