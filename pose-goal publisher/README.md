# Usage (파이썬 코드라서 별도의 make 필요없습니다)

```bash
# rosrun (패키지명) position x, y, z(실수형) orientation x, y, z, w(실수형)
rosrun pose_goal_publisher pose_pub.py 1.5 -5.6 2.3 3.3 4.4 5.5 6.6
```

pose_pub.py, goal_pub.py 입력받는 인자 갯수 동일합니다
