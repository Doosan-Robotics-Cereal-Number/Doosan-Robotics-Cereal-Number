# 🎙️ 음성 주문 시스템 - Flutter ↔ ROS2 통합 가이드

## 📌 전체 시스템 구조

```
[Flutter 키오스크 앱]
    ↓ WebSocket (ws://localhost:9090)
[rosbridge_server]
    ↓ ROS2 Topics
[voice_order_listener 노드] (항상 실행 중)
    ↓ subprocess
[main_gpt.py] (음성 주문 처리)
    ↓ stdout
[voice_order_listener 노드]
    ↓ ROS2 Topics
[rosbridge_server]
    ↓ WebSocket
[Flutter 키오스크 앱]
```

---

## 1️⃣ Flutter → ROS2 토픽 발행

### WebSocket JSON 형식

Flutter에서 rosbridge로 보내는 JSON:

```json
{
  "op": "publish",
  "topic": "/voice_order/start_voice_order",
  "msg": {
    "data": "start_voice_order"
  }
}
```

**필드 설명:**
- `op`: "publish" (토픽 발행 명령)
- `topic`: ROS2 토픽 이름
- `msg`: 메시지 내용 (std_msgs/String 타입이므로 `data` 필드 사용)

### Flutter 코드 (ros2_status_service.dart)

```dart
/// 음성 주문 시작 신호 전송
Future<void> publishVoiceOrderStart() async {
  const topic = '/voice_order/start_voice_order';
  const message = 'start_voice_order';

  // 1. 토픽 광고 (rosbridge에 토픽 등록)
  await _advertiseTopic(topic, 'std_msgs/String');
  await Future.delayed(const Duration(milliseconds: 100));

  // 2. 토픽 발행
  bool success = await publishString(topic, message);

  if (success) {
    print('[ROS2] 음성 주문 시작 신호 전송 성공');
  }
}

/// String 타입 토픽 발행 (기존 메서드)
Future<bool> publishString(String topic, String value) async {
  if (_channel == null || !_isConnected) {
    return false;
  }

  try {
    final publishMessage = jsonEncode({
      'op': 'publish',
      'topic': topic,
      'msg': {
        'data': value,
      },
    });

    _channel!.sink.add(publishMessage);
    return true;
  } catch (e) {
    print('[ROS2] 토픽 발행 실패: $e');
    return false;
  }
}
```

---

## 2️⃣ ROS2 → Flutter 토픽 구독

### WebSocket JSON 형식 (구독 요청)

Flutter가 rosbridge에 구독 요청:

```json
{
  "op": "subscribe",
  "topic": "/dsr01/kiosk/order",
  "type": "std_msgs/String"
}
```

### rosbridge가 Flutter로 보내는 메시지

```json
{
  "op": "publish",
  "topic": "/dsr01/kiosk/order",
  "msg": {
    "data": "start_sequence_a,large,personal"
  }
}
```

### Flutter 코드 (ros2_status_service.dart)

```dart
void _subscribeToOrderTopic() {
  if (_channel == null || !_isConnected) return;

  final subscribeMessage = jsonEncode({
    'op': 'subscribe',
    'topic': '/dsr01/kiosk/order',
    'type': 'std_msgs/String',
  });

  _channel!.sink.add(subscribeMessage);
  print('[ROS2] 주문 토픽 구독: /dsr01/kiosk/order');
}

/// 메시지 수신 처리
void _handleMessage(dynamic message) {
  try {
    final data = jsonDecode(message);

    if (data['op'] == 'publish') {
      if (data['topic'] == '/dsr01/kiosk/order') {
        String orderData = data['msg']['data'] ?? '';
        print('[ROS2] 주문 수신: "$orderData"');

        // 주문 데이터 파싱 및 처리
        _processOrderData(orderData);
      }
    }
  } catch (e) {
    print('[ROS2] 메시지 파싱 에러: $e');
  }
}
```

---

## 3️⃣ Flutter 앱 구조 (음성 주문 관련)

### 페이지 구조

```
welcome_page.dart
├─ "메뉴에서 선택하기" 버튼 → /cereal-selection
└─ "대화로 주문하기" 버튼 → /voice-order
                                    ↓
                            voice_order_page.dart
                            ├─ 페이지 진입 시: publishVoiceOrderStart() 호출
                            ├─ "주문 완료하기" 버튼 (테스트용)
                            └─ order_done 토픽 수신 시 → /order-complete
```

### 주요 파일

#### `lib/pages/welcome_page.dart`
```dart
// "대화로 주문하기" 버튼
ElevatedButton(
  onPressed: () {
    print('[WelcomePage] 음성 주문 시작 버튼 클릭됨');
    Navigator.pushNamed(context, '/voice-order');
  },
  child: const Text('대화로 주문하기'),
)
```

#### `lib/pages/voice_order_page.dart`
```dart
class _VoiceOrderPageState extends State<VoiceOrderPage> {
  late StatusService _statusService;

  @override
  void initState() {
    super.initState();
    _initializeService();
  }

  void _initializeService() {
    _statusService = StatusServiceFactory.create(...);
    _statusService.start();

    // TODO: 음성 주문 시작 토픽 발행 추가 필요!
    // _statusService.publishVoiceOrderStart();

    // 주문 완료 스트림 구독
    _orderDoneSubscription = _statusService.orderDoneStream.listen((done) {
      if (mounted && done) {
        Navigator.pushNamed(context, '/order-complete', arguments: orderData);
      }
    });
  }
}
```

#### `lib/services/ros2_status_service.dart`
- WebSocket 연결 관리 (ws://localhost:9090)
- 토픽 발행/구독 메서드
- 스트림 관리 (orderDoneStream, voiceOrderStartStream 등)

---

## 4️⃣ 음성 주문 시작 흐름

### Flutter 측 (구현 필요)

1. **사용자가 "대화로 주문하기" 버튼 클릭**
2. **`/voice-order` 페이지로 이동**
3. **페이지 진입 시 자동으로 토픽 발행:**

```dart
// voice_order_page.dart의 _initializeService()에 추가
await _statusService.publishVoiceOrderStart();
```

### ROS2 측 (이미 구현됨)

**토픽:** `/voice_order/start`
**타입:** `std_msgs/String`
**메시지:** `"start_voice_order"`

```python
# voice_order_listener.py
def voice_order_callback(self, msg):
    if msg.data.strip().lower() == 'start_voice_order':
        self.start_voice_order()  # main_gpt.py 실행
```

---

## 5️⃣ 주문 완료 후 Flutter로 데이터 전송

### 방법 1: 주문 내역 토픽 (현재 구현됨)

**토픽:** `/dsr01/kiosk/order`
**타입:** `std_msgs/String`
**형식:** `"start_sequence_a,medium,store"` (CSV)

```python
# main_gpt.py (음성 주문 완료 시)
print(f"[VOICE_ORDER_RESULT]{order_csv}")

# voice_order_listener.py가 stdout 모니터링 중
# [VOICE_ORDER_RESULT] 감지 → /dsr01/kiosk/order 토픽 발행
msg = String()
msg.data = order_csv  # "start_sequence_a,medium,store"
self.order_publisher.publish(msg)
```

**CSV 필드 설명:**
1. **메뉴**: `start_sequence_a` (코코볼) 또는 `start_sequence_b` (그래놀라)
2. **양**: `small` (적게), `medium` (보통), `large` (많이)
3. **컵**: `personal` (개인컵), `store` (매장컵)

### 방법 2: 주문 완료 신호 (필요 시 추가)

**토픽:** `/dsr01/kiosk/order_done`
**타입:** `std_msgs/String`
**메시지:** `"success: 'true'"` 또는 간단히 `"done"`

```dart
// Flutter에서 이미 구독 중 (ros2_status_service.dart)
void _subscribeToOrderDoneTopic() {
  final subscribeMessage = jsonEncode({
    'op': 'subscribe',
    'topic': '/dsr01/kiosk/order_done',
    'type': 'std_msgs/String',
  });

  _channel!.sink.add(subscribeMessage);
}
```

---

## 6️⃣ 전체 데이터 흐름

### 음성 주문 시작

```
1. [Flutter] "대화로 주문하기" 버튼 클릭
   ↓
2. [Flutter] /voice-order 페이지로 이동
   ↓
3. [Flutter] WebSocket으로 JSON 전송
   {
     "op": "publish",
     "topic": "/voice_order/start",
     "msg": {"data": "start_voice_order"}
   }
   ↓
4. [rosbridge] JSON → ROS2 토픽 변환
   ↓
5. [voice_order_listener] 토픽 수신 → main_gpt.py 실행
   ↓
6. [main_gpt.py] 음성 인식 + GPT 대화 진행
```

### 주문 완료 후 전송

```
7. [main_gpt.py] 주문 완료
   - 메뉴: 코코볼
   - 양: 많이
   - 컵: 개인컵
   ↓
8. [main_gpt.py] stdout 출력
   "[VOICE_ORDER_RESULT]start_sequence_a,large,personal"
   ↓
9. [voice_order_listener] stdout 모니터링 → CSV 추출
   ↓
10. [voice_order_listener] ROS2 토픽 발행
    Topic: /dsr01/kiosk/order
    Data: "start_sequence_a,large,personal"
   ↓
11. [rosbridge] ROS2 토픽 → WebSocket JSON 변환
    {
      "op": "publish",
      "topic": "/dsr01/kiosk/order",
      "msg": {"data": "start_sequence_a,large,personal"}
    }
   ↓
12. [Flutter] WebSocket 수신 → 주문 데이터 파싱
    ↓
13. [Flutter] LoadingPage → OrderCompletePage로 이동
```

---

## 7️⃣ 주요 ROS2 토픽 정리

| 토픽 이름 | 타입 | 방향 | 설명 |
|----------|------|------|------|
| `/voice_order/start_voice_order` | std_msgs/String | Flutter → ROS2 | 음성 주문 시작 신호 (`"start_voice_order"`) |
| `/dsr01/kiosk/order` | std_msgs/String | ROS2 → Flutter | 주문 내역 전송 (CSV 형식) |
| `/dsr01/kiosk/order_done` | std_msgs/String | ROS2 → Flutter | 로봇 작업 완료 신호 |

---

## 8️⃣ 개발자 체크리스트

### Flutter 측 (프론트엔드)

- [ ] `ros2_status_service.dart`에 `publishVoiceOrderStart()` 메서드 추가
- [ ] `voice_order_page.dart`의 `_initializeService()`에서 `publishVoiceOrderStart()` 호출
- [ ] `/dsr01/kiosk/order` 토픽 구독 및 주문 데이터 파싱 로직 확인
- [ ] 주문 완료 후 로딩 페이지로 이동 구현

### ROS2 측 (백엔드 - 이미 완료)

- [x] `voice_order_listener` 노드 구현
- [x] `/voice_order/start` 토픽 구독
- [x] `main_gpt.py` subprocess 실행 및 stdout 모니터링
- [x] 주문 결과 CSV 변환
- [x] `/dsr01/kiosk/order` 토픽 발행

---

## 9️⃣ 테스트 방법

### 터미널 테스트

```bash
# 터미널 1: rosbridge 실행
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 터미널 2: voice_order_listener 노드 실행
source ~/ros2_ws/install/setup.bash
ros2 run vocie_order_bridge voice_order_listener

# 터미널 3: 토픽 모니터링
ros2 topic echo /dsr01/kiosk/order

# 터미널 4: 수동 트리거 (테스트)
ros2 topic pub --once /voice_order/start_voice_order std_msgs/msg/String "data: 'start_voice_order'"
```

### Flutter 앱 테스트

1. Flutter 앱 실행
2. "대화로 주문하기" 버튼 클릭
3. 음성으로 주문 진행
4. 주문 완료 후 로딩 페이지로 자동 이동 확인

---

## 🔧 트러블슈팅

### 문제: 토픽이 발행되지 않음

**확인 사항:**
```bash
# rosbridge 실행 중인지 확인
ps aux | grep rosbridge

# 토픽 리스트 확인
ros2 topic list

# 네트워크 연결 확인
netstat -an | grep 9090
```

### 문제: Flutter에서 메시지를 받지 못함

**확인 사항:**
- WebSocket 연결 상태 확인 (`_isConnected`)
- 토픽 구독이 제대로 되었는지 확인
- rosbridge 로그 확인

### 문제: main_gpt.py가 실행되지 않음

**확인 사항:**
```bash
# 프로세스 확인
ps aux | grep main_gpt

# 수동 실행 테스트
cd ~/ros2_ws/install/vocie_order_bridge/share/vocie_order_bridge/voice_order
python3 -u main_gpt.py
```

---

## 📝 참고 사항

### ROS2 토픽 명명 규칙
- `/voice_order/start`: 음성 주문 기능 전용
- `/dsr01/kiosk/*`: 키오스크-로봇 통신 공통

### CSV 형식 변환 규칙
```python
# main_gpt.py에서 변환
menu_map = {
    "코코볼": "start_sequence_a",
    "그래놀라": "start_sequence_b"
}

size_map = {
    "적게": "small",
    "보통": "medium",
    "많이": "large"
}

cup_map = {
    "개인컵": "personal",
    "매장컵": "store"
}
```

---

**작성일:** 2025-01-14
**버전:** 1.0
**작성자:** Claude Code Assistant
