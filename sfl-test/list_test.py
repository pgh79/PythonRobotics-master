# tuple元祖 list列表

"""
a_tuple=(12,2,5,15,6)
another_tuple=2,5,6,8,3

a_list=[12,5,3,7,82]

# for content in a_tuple:
#     print(content)

for index in range(len(a_list)):
    print('index =',index,'number in list = ',a_list[index])
"""

a = [1, 2, 3, 4, 2, 3, 9, 10]

# a.append(90)# 在列表的最末尾添加一个值90
# print(a)
#
# a.insert(1,90)# 在列表的第1个位置添加一个数90
# print(a)
#
# a.remove(2)  # remove掉的不是序号，而是列表里的值，移除掉的是列表中第一次出现2的值
# print(a)
#
#
# print(a[1])
# print(a[-1])  # 索引到列表中的最后一个值
# print(a[-2])  # 索引到列表中的倒数第二个值
#
# print(a[0:3])  # 索引出列表的前三位，python中带：的索引都是左闭右开的
#
# print(a[:3])
# print(a[5:])
# print(a[-3:])


# b = a.index(2)  # 第一次出现2的时候的索引
# print(b)
# b = a.count(3)  # 计算一下3出现的次数
# print(b)
#
# a.sort()  # 不传入参数默认值时是从小到大排序，并且覆盖掉原本的a,并且没有返回参数
#
# print(a)
#
# a.sort(reverse=True)  # 将reverse参数改为True就是从大到小排序，也是覆盖掉了原本的a
#
# print(a)


"""多维列表"""

multi_dim_a=[[1,2,3],
             [2,3,4],
             [3,4,5]]
print(a[1])
print(multi_dim_a[2][2])#前面为行数，后面为列数




