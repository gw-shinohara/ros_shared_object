#include <iostream>
#include <dlfcn.h>

// Forward declarations of the external C functions from the shared object library
typedef RosSharedObjectLibrary* (*CreateNodeFunc)();
typedef void (*DestroyNodeFunc)(RosSharedObjectLibrary*);

int main(int argc, char** argv) {
    // Initialize the ROS2 system
    rclcpp::init(argc, argv);

    // Load the shared library
    void* handle = dlopen("./libros_shared_object_library.so", RTLD_LAZY);
    if (!handle) {
        std::cerr << "Cannot load library: " << dlerror() << std::endl;
        return 1;
    }

    // Reset errors
    dlerror();

    // Load the symbol for create_node
    CreateNodeFunc create_node = (CreateNodeFunc)dlsym(handle, "create_node");
    const char* dlsym_error = dlerror();
    if (dlsym_error) {
        std::cerr << "Cannot load symbol create_node: " << dlsym_error << std::endl;
        dlclose(handle);
        return 1;
    }

    // Create the node
    RosSharedObjectLibrary* node = create_node();
    if (!node) {
        std::cerr << "Failed to create the node" << std::endl;
        dlclose(handle);
        return 1;
    }

    // Test the publish_message function
    node->publish_message("Hello, ROS2 from shared object!");

    // Test the start function (allow some time to process)
    node->start();

    // Wait for a few seconds to ensure messages are published
    rclcpp::sleep_for(std::chrono::seconds(2));

    // Load the symbol for destroy_node
    DestroyNodeFunc destroy_node = (DestroyNodeFunc)dlsym(handle, "destroy_node");
    dlsym_error = dlerror();
    if (dlsym_error) {
        std::cerr << "Cannot load symbol destroy_node: " << dlsym_error << std::endl;
        dlclose(handle);
        return 1;
    }

    // Destroy the node
    destroy_node(node);

    // Close the library
    dlclose(handle);

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}
