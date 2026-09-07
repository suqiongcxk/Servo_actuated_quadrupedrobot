#include "main.h"
#include "stdio.h"
#include "my_assert.h"

// 断言失败处理函数
void Assert_Failed(const char *file, int line)
{
    // 卡死循环
		printf("断言失败在: %s 第%d行\n", file, line);
    while (1)
    {
        // 可以在这里添加LED闪烁
				
    }
}


