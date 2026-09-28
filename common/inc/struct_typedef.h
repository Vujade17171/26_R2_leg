#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

#include <stdint.h>   // 使用标准整数类型
#include <stdbool.h>  // 使用标准bool类型

// 如果需要项目特定的类型别名
typedef float fp32;
typedef double fp64;

// 如果必须保留bool_t（建议直接用bool）
#ifndef bool_t
#define bool_t bool
#endif

#endif



