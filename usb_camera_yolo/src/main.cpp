#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/stat.h> // for mkfifo
#include <fcntl.h>    // for open flags
#include <csignal>    // for signal handling

// 全局变量用于信号处理时关闭子进程
pid_t global_child_pid = -1;

// 捕获 Ctrl+C，确保子进程被杀死
void signalHandler(int signum)
{
    if (global_child_pid > 0)
    {
        std::cout << "\nTerminating Python process..." << std::endl;
        kill(global_child_pid, SIGTERM);
        waitpid(global_child_pid, nullptr, 0);
    }
    exit(signum);
}

int main()
{
    std::string pipe_path = "/tmp/my_pipe";

    // 1. 【改进】由 C++ 创建管道，确保文件存在
    // 0666 表示读写权限。如果文件已存在 mkfifo 会失败，但没关系
    if (mkfifo(pipe_path.c_str(), 0666) == -1 && errno != EEXIST)
    {
        perror("Failed to create pipe");
        return 1;
    }

    pid_t pid = fork();

    if (pid == -1)
    {
        std::cerr << "Failed to fork" << std::endl;
        return 1;
    }
    else if (pid == 0)
    {
        // === 子进程 (Python) ===
        // 这里的路径建议使用绝对路径，或者确保当前工作目录正确
        execlp("/usr/local/miniconda3/bin/python", "python_gesture_worker",
               "/home/HwHiAiUser/yhy_test/car/usb_camera_yolo/py/named_pipes.py", NULL);
        perror("execlp failed"); // 如果 execlp 返回，打印错误
        _exit(1);
    }
    else
    {
        // === 父进程 (C++) ===
        global_child_pid = pid;

        // 注册信号处理，防止 Ctrl+C 后 Python 还在跑
        signal(SIGINT, signalHandler);

        std::cout << "Python script started with PID: " << pid << std::endl;
        std::cout << "Waiting for pipe connection..." << std::endl;

        // 2. 打开管道
        // std::ifstream 打开管道会阻塞，直到写端（Python）也打开管道
        // 这是预期的同步行为
        std::ifstream pipe(pipe_path);

        if (!pipe.is_open())
        {
            std::cerr << "Failed to open pipe" << std::endl;
            kill(pid, SIGTERM);
            return 1;
        }

        std::string line;
        while (std::getline(pipe, line))
        {
            // 这里处理接收到的数据
            // std::cout << "Received: " << line << std::endl;

            // 如果你需要解析整数：
            try
            {
                int gesture_id = std::stoi(line);
                std::cout << "Gesture from python: " << gesture_id << std::endl;
            }
            catch (...)
            {
                // 忽略解析错误
            }
        }

        std::cout << "Pipe closed by writer (Python ended)." << std::endl;

        // 3. 正常退出时的回收
        kill(pid, SIGTERM); // 确保子进程真的结束了
        waitpid(pid, nullptr, 0);

        // 可选：删除管道文件
        unlink(pipe_path.c_str());
    }
    return 0;
}
