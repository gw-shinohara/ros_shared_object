#include <dlfcn.h>
#include <cstdint>
#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>

using namespace std;

namespace
{
    const string SO_FILE_PATH = "/home/ros_shared_object/build/libros_shared_object_library.so";
    volatile sig_atomic_t flag = 1;

    void handler(int signum)
    {
        flag = 0;
    }
}

int main(int argc, char* argv[])
{
    signal(SIGINT, handler);

    void* handle = dlopen(SO_FILE_PATH.c_str(), RTLD_NOW);
    if (!handle) {
        cerr << "Error loading shared object: " << dlerror() << endl;
        return 1;
    }

    intptr_t (*create)() = (intptr_t (*)())dlsym(handle, "create");
    if (!create) {
        cerr << "Error loading symbol create: " << dlerror() << endl;
        return 1;
    }

    void (*spin_some)(intptr_t) = (void (*)(intptr_t))dlsym(handle, "spin_some");
    if (!spin_some) {
        cerr << "Error loading symbol spin_some: " << dlerror() << endl;
        return 1;
    }

    void (*talk)(intptr_t, int) = (void (*)(intptr_t, int))dlsym(handle, "talk");
    if (!talk) {
        cerr << "Error loading symbol talk: " << dlerror() << endl;
        return 1;
    }

    
    void (*destroy)(intptr_t) = (void (*)(intptr_t))dlsym(handle, "destroy");
    if (!destroy) {
        cerr << "Error loading symbol destroy: " << dlerror() << endl;
        return 1;
    }
    intptr_t instance = (*create)();

    while (flag != 0)
    {
        (*talk)(instance, 0);
        (*spin_some)(instance);
        this_thread::sleep_for(chrono::seconds(1));
    }

    (*destroy)(instance);

    dlclose(handle);

    return 0;
}
