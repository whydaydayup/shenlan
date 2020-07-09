动态内存new和delete

之前的程序中，所必需的内存空间的大小都是在程序执行之前就已经确定了。但是如果我们需要的内存是一个变量，数值只有在程序运行时才能确定，例如：需要根据用户的输入来决定内存空间。答案是动态内存分配(dynamic memory)，为此C++ 集成了操作符new 和delete。

1、简单的数值与数组内存

new的用法：new在动态内存堆中为对象分配空间并返回一个指向该对象的指针。

     new 数据类型 ;                  //申请内存空间。
     new 数据类型   (初值)；          //申请内存空间时，并指定该数据类型的初值。
     new 数据类型   [内存单元个数];    //申请多个内存空间。
    
    int* a = new int (4);   //开辟一个int类型指针赋值给a，并地址中的内容赋值为4。
    delete a;               //释放单个int的空间
    int* b = new int[100];  //开辟一个大小为100的整型数组空间。
    delete [] a;            //释放int数组空间

缺点： 有时我们会忘记释放内存，这样就导致内存泄漏

 2、智能指针动态管理内存

shared_ptr允许多个指针指向同一个对象，unique则是独占所指向的对象，weak_ptr是一种弱引用，指向shared_ptr管理的对象。

（1）智能指针是模板，当我们创建一个智能指针时候，必须提供额外信息------指针可以指向的类型，与vector一样，我们在<>中给出类型，之后就是所定义的这种智能指针的名字：

shared_ptr<string> p1;//shared_ptr可以指向string。

（2）shared_ptr与new结合

shared_ptr<int> p1(new int(1024))//直接初始化

相当于new返回的int* 来创建一个shared_ptr<int> 来。

    导入照片：      
    
    std::stringstream ss;
    ss <<  "F:/SLAM/dataset/image_1/"<< std::setw(6) << std::setfill('0') << img_id<< ".png";
    cv::Mat img(cv::imread(ss.str().c_str(), 0));
    assert(!img.empty());//如果没有照片就返回错误，终止程序

其中setw(6)表示图片名称是6个字符长，从右开始数，多余部分用0补充，例如：000006.png

ss.str().c_str()将string类型转为char*，为了与C兼容。