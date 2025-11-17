# Flutter 코드 수정 사항 - 음성 주문 통합

## 📍 수정 파일
`lib/services/ros2_status_service.dart`

---

## ✅ 수정 사항

### 1️⃣ 토픽 구독 호출 추가 (90-92줄)

**현재 코드:**
```dart
// 토픽 구독
_subscribeToTopic();
_subscribeToOrderDoneTopic();
```

**수정 후:**
```dart
// 토픽 구독
_subscribeToTopic();
_subscribeToOrderDoneTopic();
_subscribeToOrderTopic();  // ← 이 줄 추가!
```

---

### 2️⃣ 주문 정보 구독 함수 추가 (142줄 아래에 추가)

`_subscribeToOrderDoneTopic()` 함수 아래에 다음 함수를 추가:

```dart
/// 주문 정보 토픽 구독
void _subscribeToOrderTopic() {
  if (_channel == null || !_isConnected) return;

  final subscribeMessage = jsonEncode({
    'op': 'subscribe',
    'topic': '/dsr01/kiosk/order',
    'type': 'std_msgs/String',
  });

  _channel!.sink.add(subscribeMessage);
  print('[ROS2] 주문 정보 토픽 구독: /dsr01/kiosk/order');
}
```

---

### 3️⃣ 주문 정보 처리 로직 추가 (_handleMessage 함수 내부, 213줄 아래)

**위치:** `order_done` 처리 코드 블록이 끝나는 `}` 다음에 추가

```dart
// 주문 완료 토픽 처리
else if (data['topic'] == '/dsr01/kiosk/order_done') {
  // ... 기존 코드 ...
}
// 주문 정보 토픽 처리 ← 여기부터 추가!
else if (data['topic'] == '/dsr01/kiosk/order') {
  String orderCsv = data['msg']['data'] ?? '';
  print('[ROS2] ✅ 주문 정보 수신: "$orderCsv"');

  // CSV 파싱
  List<String> parts = orderCsv.split(',');
  if (parts.length == 3) {
    // 한글 변환
    String menu = parts[0].contains('sequence_a') ? '코코볼' : '그래놀라';
    String size = parts[1] == 'large' ? '많이' :
                  parts[1] == 'small' ? '적게' : '보통';
    String cup = parts[2] == 'personal' ? '개인컵' : '매장컵';

    print('[ROS2] 📋 주문 내역: 메뉴=$menu, 양=$size, 컵=$cup');

    // TODO: OrderData 생성해서 화면에 전달
    // 필요하면 StreamController 추가해서 UI로 전달
  }
}
```

---

## 📋 데이터 형식

### ROS2에서 보내는 메시지
```json
{
  "op": "publish",
  "topic": "/dsr01/kiosk/order",
  "msg": {
    "data": "start_sequence_a,large,personal"
  }
}
```

### CSV 형식 (쉼표로 구분)
```
메뉴,양,컵
```

**예시:**
- `start_sequence_a,large,personal` → 코코볼, 많이, 개인컵
- `start_sequence_b,medium,store` → 그래놀라, 보통, 매장컵

### 변환 규칙

| CSV 값 | 한글 |
|--------|------|
| **메뉴** | |
| `start_sequence_a` | 코코볼 |
| `start_sequence_b` | 그래놀라 |
| **양** | |
| `small` | 적게 |
| `medium` | 보통 |
| `large` | 많이 |
| **컵** | |
| `personal` | 개인컵 |
| `store` | 매장컵 |

---

## 🔄 전체 흐름

```
1. 사용자가 음성으로 주문
   ↓
2. ROS2 노드가 음성 인식 + GPT 처리
   ↓
3. ROS2 토픽 발행: /dsr01/kiosk/order
   데이터: "start_sequence_a,large,personal"
   ↓
4. rosbridge가 WebSocket JSON으로 변환
   ↓
5. Flutter가 WebSocket 수신
   ↓
6. _handleMessage()에서 파싱
   ↓
7. 화면에 "코코볼, 많이, 개인컵" 표시
```

---

## 🧪 테스트 방법

### 1. ROS2 노드 실행
```bash
ros2 run vocie_order_bridge voice_order_listener
```

### 2. rosbridge 실행
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 3. Flutter 앱 실행
```bash
flutter run -d linux
```

### 4. 수동 토픽 발행 (테스트용)
```bash
ros2 topic pub --once /dsr01/kiosk/order std_msgs/msg/String "data: 'start_sequence_a,large,personal'"
```

Flutter 콘솔에 다음과 같이 출력되어야 함:
```
[ROS2] ✅ 주문 정보 수신: "start_sequence_a,large,personal"
[ROS2] 📋 주문 내역: 메뉴=코코볼, 양=많이, 컵=개인컵
```

---

## 📞 문의사항

추가로 필요한 기능이나 수정 사항이 있으면 백엔드팀에 연락주세요!
