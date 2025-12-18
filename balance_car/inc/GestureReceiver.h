//
// Created by yangh on 2025/12/18.
//

#ifndef BALANCECAR_GESTURERECEIVER_H
#define BALANCECAR_GESTURERECEIVER_H


#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <atomic>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <csignal>

class GestureReceiver
{
public:
    // 构造函数：设定管道路径和 Python 脚本路径
    GestureReceiver(const std::string& pipe_path,
                    const std::string& python_interpreter,
                    const std::string& script_path)
        : m_pipe_path(pipe_path),
          m_python_interpreter(python_interpreter),
          m_script_path(script_path),
          m_child_pid(-1),
          m_running(false),
          m_gesture_id(-1) // -1 表示无效或未接收到
    {
    }

    // 析构函数：确保对象销毁时停止线程和子进程
    ~GestureReceiver()
    {
        stop();
    }

    // 启动子进程和接收线程
    bool start()
    {
        if (m_running) return true; // 已经在运行

        // 1. 创建命名管道
        if (mkfifo(m_pipe_path.c_str(), 0666) == -1 && errno != EEXIST)
        {
            perror("Failed to create pipe");
            return false;
        }

        // 2. Fork 进程
        pid_t pid = fork();
        if (pid == -1)
        {
            perror("Failed to fork");
            return false;
        }
        else if (pid == 0)
        {
            // === 子进程 (Python) ===
            // 建议使用 execvp 或者 execlp，注意参数
            execlp(m_python_interpreter.c_str(),
                   "python_gesture_worker", // argv[0] 名字
                   m_script_path.c_str(),
                   NULL);

            perror("execlp failed");
            _exit(1); // 失败直接退出子进程
        }
        else
        {
            // === 父进程 ===
            m_child_pid = pid;
            m_running = true;

            // 3. 开启后台线程读取管道
            m_worker_thread = std::thread(&GestureReceiver::readLoop, this);

            std::cout << "[GestureReceiver] Started. Python PID: " << m_child_pid << std::endl;
        }
        return true;
    }

    // 停止线程和子进程
    void stop()
    {
        if (!m_running) return;

        m_running = false; // 设置标志位

        // 1. 杀死 Python 子进程
        // 这非常重要：杀死写端（Python），读端（C++ pipe）才会收到 EOF 并结束 getline 阻塞
        if (m_child_pid > 0)
        {
            std::cout << "[GestureReceiver] Killing Python process..." << std::endl;
            kill(m_child_pid, SIGTERM);
            waitpid(m_child_pid, nullptr, 0); // 等待回收僵尸进程
            m_child_pid = -1;
        }

        // 2. 等待读取线程结束
        if (m_worker_thread.joinable())
        {
            m_worker_thread.join();
        }

        // 3. 清理管道文件（可选）
        unlink(m_pipe_path.c_str());
        std::cout << "[GestureReceiver] Stopped." << std::endl;
    }

    // 获取最新的手势 ID (线程安全)
    int getGestureId() const
    {
        return m_gesture_id.load();
    }

private:
    std::string m_pipe_path;
    std::string m_python_interpreter;
    std::string m_script_path;

    pid_t m_child_pid;
    std::thread m_worker_thread;
    std::atomic<bool> m_running; // 控制线程运行
    std::atomic<int> m_gesture_id; // 存储最新的 ID，原子操作

    // 线程执行的循环函数
    void readLoop()
    {
        std::ifstream pipe(m_pipe_path);
        if (!pipe.is_open())
        {
            std::cerr << "[GestureReceiver] Failed to open pipe in thread." << std::endl;
            return;
        }

        std::string line;
        // 当 Python 进程被 kill 时，管道写入端关闭，getline 会返回 false，循环自动结束
        while (m_running && std::getline(pipe, line))
        {
            try
            {
                if (!line.empty())
                {
                    // 更新原子变量
                    m_gesture_id.store(std::stoi(line));
                    // 调试输出（可选）
                    // std::cout << "[Thread] Updated gesture: " << line << std::endl;
                }
            }
            catch (...)
            {
                // 忽略解析错误
            }
        }
        pipe.close();
    }
};


#endif //BALANCECAR_GESTURERECEIVER_H
