#include "myslam/config.h"

namespace myslam 
{
// 一个静态成员函数,无返回值,
// &在变量定义区, 表示引用, 要注意它的用法, int &x;
// &在变量操作区, 表示取地址符, int x=10, *p=&x ;  //这里&作用在x上， 是取地址符
// const std::string&可以绑定到一个普通string&上
void Config::setParameterFile( const std::string& filename )
{
    // 动态指针默认会被初始化为空指针nullptr
    if ( config_ == nullptr ) // 智能指针为空nullptr空指针
        // 调用私有的默认构造函数,创建一个智能指针指向Config类, new Config在自由空间内分配内存,new无法为其分配的对象命名,而是返回一个指向该对象的指针
        // new Config创建一个Config类的对象,并返回指向这个新创建的Config类的对象的指针
        // 用new返回的指针初始化智能指针
        config_ = shared_ptr<Config>(new Config);
    // config_动态指针, 类中的成员file_负责打开文件
    // cv::FileStorage(string fileName, flag)给定文件名,创建一个cv::FileStorage对象读取一个XML或YAML格式的数据文件,
    // c_str()为了与c语言兼容，在c语言中没有string类型，故必须通过string类对象的成员函数c_str()把string对象转换成c中的字符串样式,它返回当前字符串的首字符地址。
    // config_->file_ = cv::FileStorage( filename.c_str(), cv::FileStorage::READ );
    config_->file_ = cv::FileStorage( filename, cv::FileStorage::READ );
    if ( config_->file_.isOpened() == false ) // 检查文件是否打开成功
    {
        // 文件打开失败,报错cerr
        // cerr默认情况下被关联到标准输出流，但不经过缓冲而直接输出，一般用于迅速输出出错信息，是标准错误，
        std::cerr<<"parameter file "<<filename<<" does not exist."<<std::endl;
        config_->file_.release(); // 关闭文件并释放存储缓存Closes the file and releases all the memory buffers.
        return;
    }
}

Config::~Config() // Config类的销毁
{
    if ( file_.isOpened() ) // 若文件被打开
        file_.release(); // 关闭文件并释放存储缓存
}
// 将Config类中的config_静态成员-动态指针初始化为nullptr空指针
shared_ptr<Config> Config::config_ = nullptr;
}