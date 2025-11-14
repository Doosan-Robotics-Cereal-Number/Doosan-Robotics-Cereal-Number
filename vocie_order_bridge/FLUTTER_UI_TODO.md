# Flutter UI 추가 작업 - voice_order_page

## 📍 현재 상태

✅ ROS2에서 주문 정보 수신 성공
✅ CSV 파싱 및 한글 변환 성공
❌ UI에 표시하는 기능 필요

---

## 🔧 작업 1: ros2_status_service.dart 수정

### 1-1. StreamController 추가 (17줄 아래)

```dart
// 스트림 컨트롤러
final _statusStreamController = StreamController<int>.broadcast();
final _connectionStreamController = StreamController<bool>.broadcast();
final _orderDoneStreamController = StreamController<bool>.broadcast();
final _orderInfoStreamController = StreamController<Map<String, String>>.broadcast();  // ← 추가!
```

### 1-2. Stream getter 추가 (기존 Stream getter들 아래)

```dart
Stream<Map<String, String>> get orderInfoStream => _orderInfoStreamController.stream;
```

### 1-3. dispose()에 StreamController 닫기 추가

```dart
@override
void dispose() {
  _statusStreamController.close();
  _connectionStreamController.close();
  _orderDoneStreamController.close();
  _orderInfoStreamController.close();  // ← 추가!
}
```

### 1-4. _handleMessage()에서 주문 정보 스트림 발행 (현재 print 부분 수정)

**현재 코드:**
```dart
else if (data['topic'] == '/dsr01/kiosk/order') {
  String orderCsv = data['msg']['data'] ?? '';
  print('[ROS2] ✅ 주문 정보 수신: "$orderCsv"');

  List<String> parts = orderCsv.split(',');
  if (parts.length == 3) {
    String menu = parts[0].contains('sequence_a') ? '코코볼' : '그래놀라';
    String size = parts[1] == 'large' ? '많이' :
                  parts[1] == 'small' ? '적게' : '보통';
    String cup = parts[2] == 'personal' ? '개인컵' : '매장컵';

    print('[ROS2] 📋 주문 내역: 메뉴=$menu, 양=$size, 컵=$cup');

    // TODO: OrderData 생성해서 화면에 전달
  }
}
```

**수정 후:**
```dart
else if (data['topic'] == '/dsr01/kiosk/order') {
  String orderCsv = data['msg']['data'] ?? '';
  print('[ROS2] ✅ 주문 정보 수신: "$orderCsv"');

  List<String> parts = orderCsv.split(',');
  if (parts.length == 3) {
    String menu = parts[0].contains('sequence_a') ? '코코볼' : '그래놀라';
    String size = parts[1] == 'large' ? '많이' :
                  parts[1] == 'small' ? '적게' : '보통';
    String cup = parts[2] == 'personal' ? '개인컵' : '매장컵';

    print('[ROS2] 📋 주문 내역: 메뉴=$menu, 양=$size, 컵=$cup');

    // 주문 정보를 스트림으로 전달
    _orderInfoStreamController.add({
      'menu': menu,
      'size': size,
      'cup': cup,
    });
  }
}
```

---

## 🎨 작업 2: voice_order_page.dart 수정

### 2-1. 주문 정보 상태 변수 추가

```dart
class _VoiceOrderPageState extends State<VoiceOrderPage> {
  late StatusService _statusService;
  StreamSubscription<bool>? _orderDoneSubscription;
  StreamSubscription<Map<String, String>>? _orderInfoSubscription;  // ← 추가!
  OrderData? orderData;

  // 주문 정보 표시용 상태 변수 ← 추가!
  String? _receivedMenu;
  String? _receivedSize;
  String? _receivedCup;
```

### 2-2. initState()에서 주문 정보 스트림 구독

```dart
void _initializeService() {
  _statusService = StatusServiceFactory.create(...);
  _statusService.start();

  // 주문 완료 스트림 구독
  _orderDoneSubscription = _statusService.orderDoneStream.listen((done) {
    // ... 기존 코드
  });

  // 주문 정보 스트림 구독 ← 추가!
  _orderInfoSubscription = _statusService.orderInfoStream.listen((orderInfo) {
    if (mounted) {
      setState(() {
        _receivedMenu = orderInfo['menu'];
        _receivedSize = orderInfo['size'];
        _receivedCup = orderInfo['cup'];
      });
      print('[VoiceOrderPage] 주문 정보 수신: 메뉴=$_receivedMenu, 양=$_receivedSize, 컵=$_receivedCup');
    }
  });
}
```

### 2-3. dispose()에 구독 취소 추가

```dart
@override
void dispose() {
  _orderDoneSubscription?.cancel();
  _orderInfoSubscription?.cancel();  // ← 추가!
  _statusService.stop();
  _statusService.dispose();
  super.dispose();
}
```

### 2-4. UI에 주문 정보 표시

**build() 메서드 내부, "주문 완료하기" 버튼 위에 추가:**

```dart
@override
Widget build(BuildContext context) {
  return Scaffold(
    backgroundColor: Colors.white,
    body: SafeArea(
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 40.0, vertical: 30.0),
        child: Column(
          children: [
            // 뒤로가기 버튼 (기존 코드)
            // ...

            const SizedBox(height: 60),

            // 메인 컨텐츠 영역
            const Spacer(),

            // 주문 정보 표시 ← 여기 추가!
            if (_receivedMenu != null && _receivedSize != null && _receivedCup != null)
              Container(
                padding: const EdgeInsets.all(40),
                decoration: BoxDecoration(
                  color: Colors.white,
                  borderRadius: BorderRadius.circular(20),
                  border: Border.all(
                    color: const Color(0xFF0064FF),
                    width: 2,
                  ),
                ),
                child: Column(
                  children: [
                    const Text(
                      '주문 내역',
                      style: TextStyle(
                        fontSize: 32,
                        fontWeight: FontWeight.bold,
                        color: Color(0xFF121212),
                      ),
                    ),
                    const SizedBox(height: 30),
                    _buildOrderInfoRow('메뉴', _receivedMenu!),
                    const SizedBox(height: 16),
                    _buildOrderInfoRow('양', _receivedSize!),
                    const SizedBox(height: 16),
                    _buildOrderInfoRow('컵', _receivedCup!),
                  ],
                ),
              ),

            const Spacer(),

            // 주문 완료 버튼 (기존 코드)
            // ...
          ],
        ),
      ),
    ),
  );
}

// 주문 정보 행 위젯 ← 추가!
Widget _buildOrderInfoRow(String label, String value) {
  return Row(
    mainAxisAlignment: MainAxisAlignment.center,
    children: [
      Text(
        '$label: ',
        style: const TextStyle(
          fontSize: 24,
          color: Color(0xFF666666),
        ),
      ),
      Text(
        value,
        style: const TextStyle(
          fontSize: 24,
          fontWeight: FontWeight.w600,
          color: Color(0xFF0064FF),
        ),
      ),
    ],
  );
}
```

---

## 📋 예상 화면

```
┌─────────────────────────────┐
│  뒤로가기                     │
│                              │
│                              │
│     ┌──────────────┐         │
│     │  주문 내역    │         │
│     │              │         │
│     │ 메뉴: 그래놀라 │         │
│     │ 양: 많이      │         │
│     │ 컵: 매장컵    │         │
│     └──────────────┘         │
│                              │
│                              │
│  ┌────────────────┐          │
│  │ 주문 완료하기    │          │
│  └────────────────┘          │
└─────────────────────────────┘
```

---

## ✅ 체크리스트

### ros2_status_service.dart
- [ ] `_orderInfoStreamController` 추가
- [ ] `orderInfoStream` getter 추가
- [ ] `dispose()`에 close 추가
- [ ] `_handleMessage()`에서 스트림 발행

### voice_order_page.dart
- [ ] 상태 변수 추가 (`_receivedMenu`, `_receivedSize`, `_receivedCup`)
- [ ] `orderInfoStream` 구독
- [ ] `dispose()`에 구독 취소 추가
- [ ] UI에 주문 정보 표시 위젯 추가

---

## 🧪 테스트

1. Flutter 앱 실행
2. "대화로 주문하기" 클릭
3. 음성으로 주문 (예: "그래놀라 많이 매장컵으로 주세요")
4. 주문 완료되면 화면에 "메뉴: 그래놀라, 양: 많이, 컵: 매장컵" 표시되어야 함

---

**작성일:** 2025-01-14
**버전:** 1.0
