#ifndef MY_ASSERT_H
#define MY_ASSERT_H

// 断言开关 (1=开, 0=关)
#define ASSERT_ENABLED 1

#if ASSERT_ENABLED
    // 断言宏
    #define ASSERT(expr) \
        if (!(expr)) { \
            Assert_Failed(__FILE__, __LINE__); \
        }
#else
    // 关闭时生成空代码
    #define ASSERT(expr) ((void)0)
#endif

// 断言失败处理函数声明
void Assert_Failed(const char *file, int line);

#endif /* MY_ASSERT_H */
				
				
				