#### log内容合并，不要通过鼠标操作

采集到数据通过betaflight-log-viewer显示时，需要添加头部分

head -n 1.txt > 2.txt   将1.txt的前面n行复制到2.txt

sed -n 'x,yp' 1.txt >> 2.txt  将1.txt从x到y行的内容添加到2.txt的后面