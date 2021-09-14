#pragma once;
#define DUMP_I(x) { \
  Serial.printf("%s::%d:%s=<%d>\r\n",__func__,__LINE__,#x,x);\
}
#define DUMP_F(x) { \
  Serial.printf("%s::%d:%s=<%f>\r\n",__func__,__LINE__,#x,x);\
}
#define DUMP_S(x) { \
  Serial.printf("%s::%d:%s=<%s>\r\n",__func__,__LINE__,#x,x.c_str());\
}