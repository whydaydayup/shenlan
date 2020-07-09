// Config类负责参数文件的读取,在程序的任何位置都能可随时提供参数的值,
// 把config写成单文件singleton模式
// 只有一个全局对象,当我们设置参数文件时,创建该对象并读取参数文件,
// 随后就可以在任意地方访问参数值,最后在程序结束时自动销毁
#ifndef CONFIG_H
#define CONFIG_H

#include "myslam/common_include.h" 

namespace myslam 
{
class Config
{
private:
    // 静态成员是可以独立访问的，也就是说，无须创建任何对象实例就可以访问，静态成员函数可不建立对象就可以被使用。
    // 指向Config类的静态智能指针config_
    // 不初始化一个动态指针,就会被初始化为一个空指针
    static std::shared_ptr<Config> config_;

    cv::FileStorage file_;// cv::FileStorage类的对象file_

    // 构造函数声明为私有,防止这个类的对象在别处建立,它只能在 setParameterFile 时构造。
    // 实际构造的对象是 Config 的智能指针:static shared_ptr<Config> config_.
    // 智能指针可以自动析构,省得再调一个别的函数来析构
    Config () {} // private constructor makes a singleton
    // 私有的构造函数,单例模式singleton:
    // 1.保证一个类只创建一个实例, 2.提供对该实例的全局访问点
    // 保证整个程序中只有一个全局对象，设置参数文件时被创建，程序结束时被销毁。

public:
    ~Config();  // close the file when deconstructing
    // 公有的销毁函数, 销毁对象时,关闭文件


    // set a new config file创建一个新的私有配置文件
    // static静态成员,可使用作用域运算符直接访问静态成员
    static void setParameterFile( const std::string& filename ); 

    // 文件读取方面使用 OpenCV 提供的 FileStorage 类。它可以读取一个 YAML文件,且可以访问其中任意一个字段。
    // 由于参数实质值可能为整数、浮点数或字符串, 所以通过一个模板函数 get,来获得任意类型的参数值。
    // access the parameter values获得相应的键值对
    // 通用的函数模板function template, template关键字,<typename T>是一个模板参数列表,名为T的类型参数,用T表示一个类型
    // 通过[key]获取相应的值value,键值对的形式存储数据
    template< typename T >
    static T get( const std::string& key ) // &在变量定义区, 表示引用,
    {
        return T( Config::config_->file_[key] );
    }
};
}
#endif // CONFIG_H