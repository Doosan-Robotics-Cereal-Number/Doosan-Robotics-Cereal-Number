# 음성 주문 UI 업데이트 가이드

## 📋 개요

음성 주문 기능의 UI 흐름을 개선하여, 주문 내역 표시 화면을 `LoadingPage`와 동일한 화면으로 통일했습니다. 또한 개발 및 테스트를 위한 테스트 버튼을 추가했습니다.

## 🎯 주요 변경 사항

### 1. 주문 내역 화면 통일

**변경 전:**
- `VoiceOrderPage`에서 주문 정보를 받으면 텍스트 형태의 주문 내역을 표시
- 주문 완료 후 `OrderCompletePage`로 이동

**변경 후:**
- `VoiceOrderPage`에서 주문 정보를 받으면 `VoiceOrderLoadingPage`로 이동
- `VoiceOrderLoadingPage`는 `LoadingPage` 위젯을 재사용하여 동일한 UI 표시
- 주문 완료 후 `OrderCompletePage`로 이동

### 2. 테스트 버튼 추가

개발 및 테스트를 위해 랜덤 주문 데이터를 생성하여 ROS2 토픽에 직접 발행하는 테스트 버튼을 추가했습니다.

## 📁 변경된 파일

### 새로 생성된 파일

#### 1. `cereal_order_app/lib/pages/voice_order_loading_page.dart`

음성 주문 전용 로딩 페이지입니다. `LoadingPage` 위젯을 재사용하여 주문 내역을 시각적으로 표시합니다.

**주요 기능:**
- `LoadingPage` 위젯 재사용
- 주문 완료 스트림(`orderDoneStream`) 구독
- 주문 완료 시 `OrderCompletePage`로 자동 이동

**사용 방법:**
```dart
Navigator.pushNamed(
  context,
  '/voice-order-loading',
  arguments: {
    'orderData': orderData,
    'statusService': statusService,
  },
);
```

#### 2. `cereal_order_app/lib/widgets/voice_order_test_button.dart`

음성 주문 테스트용 버튼 위젯입니다. `AppConfig.debugMode`가 `true`일 때만 표시됩니다.

**주요 기능:**
- 랜덤 주문 데이터를 ROS2 CSV 형식으로 생성
- 형식: `"start_sequence_a,medium,store"`
- 콜백을 통해 생성된 CSV 문자열 전달

**랜덤 생성 옵션:**
- 메뉴: `start_sequence_a` (코코볼) 또는 `start_sequence_b` (그래놀라)
- 양: `small` (적게), `medium` (보통), `large` (많이)
- 컵: `personal` (개인컵) 또는 `store` (매장컵)

### 수정된 파일

#### 1. `cereal_order_app/lib/pages/voice_order_page.dart`

**주요 변경 사항:**

1. **주문 내역 UI 제거**
   - 기존의 텍스트 형태 주문 내역 표시 UI 제거
   - Pulsator 애니메이션만 유지 (주문 정보 수신 대기 중)

2. **주문 정보 수신 시 처리 변경**
   ```dart
   _orderInfoSubscription = (_statusService as dynamic).orderInfoStream.listen((orderInfo) {
     // OrderData로 변환
     OrderData voiceOrderData = _convertToOrderData(...);
     
     // voice_order_loading_page로 이동
     Navigator.pushNamed(context, '/voice-order-loading', arguments: {...});
   });
   ```

3. **테스트 버튼 추가**
   - `AppConfig.debugMode`가 `true`일 때만 표시
   - 버튼 클릭 시 ROS2 토픽에 직접 발행

4. **테스트 주문 처리 로직**
   ```dart
   Future<void> _onTestOrderGenerated(String orderCsv) async {
     // 1. CSV 파싱하여 OrderData 생성
     // 2. ROS2 연결 상태 확인
     // 3. 토픽 발행 시도
     // 4. 발행 성공 → orderInfoStream 리스너가 처리
     // 5. 발행 실패 → /loading으로 직접 이동
   }
   ```

#### 2. `cereal_order_app/lib/main.dart`

**주요 변경 사항:**

1. **새 라우트 추가**
   ```dart
   '/voice-order-loading': (context) {
     final args = ModalRoute.of(context)?.settings.arguments as Map<String, dynamic>?;
     if (args != null && args['orderData'] != null && args['statusService'] != null) {
       return VoiceOrderLoadingPage(
         orderData: args['orderData'] as OrderData,
         statusService: args['statusService'] as StatusService,
       );
     }
     // 에러 처리
   }
   ```

2. **필요한 import 추가**
   - `voice_order_loading_page.dart`
   - `order_data.dart`
   - `status_service.dart`

## 🔄 동작 흐름

### 실제 음성 주문 흐름

```
1. VoiceOrderPage 진입
   ↓
2. 음성 주문 시작 신호 발행 (/dsr01/kiosk/start_voice_order)
   ↓
3. 로봇 AI 애니메이션 표시 (주문 대기 중)
   ↓
4. 주문 정보 수신 (/dsr01/kiosk/order 토픽)
   ↓
5. VoiceOrderLoadingPage로 이동
   - LoadingPage 위젯 재사용
   - 주문 내역 시각적 표시
   ↓
6. 주문 완료 신호 수신 (/dsr01/kiosk/order_done 토픽)
   ↓
7. OrderCompletePage로 이동
```

### 테스트 버튼 흐름

```
1. VoiceOrderPage에서 테스트 버튼 클릭
   ↓
2. 랜덤 CSV 생성 (예: "start_sequence_a,medium,store")
   ↓
3. ROS2 연결 상태 확인
   ├─ 연결 안 됨 → /loading으로 직접 이동
   └─ 연결됨 → 다음 단계
   ↓
4. ROS2 토픽에 주문 정보 발행 (/dsr01/kiosk/order)
   ├─ 발행 성공 → orderInfoStream 리스너가 처리
   │                → VoiceOrderLoadingPage로 이동
   └─ 발행 실패 → /loading으로 직접 이동
   ↓
5. 이후 흐름은 실제 음성 주문과 동일
```

## 🛠️ 설정

### 테스트 버튼 표시/숨김

`cereal_order_app/lib/config/app_config.dart`에서 설정:

```dart
/// 디버그 모드 (true면 테스트 버튼 표시)
static const bool debugMode = true;  // false로 변경하면 버튼 숨김
```

## 📊 데이터 형식

### ROS2 토픽 형식

**주문 정보 토픽:** `/dsr01/kiosk/order`
- 타입: `std_msgs/String`
- 형식: CSV 문자열
- 예시: `"start_sequence_a,medium,store"`

**필드 설명:**
- 첫 번째 필드: 메뉴
  - `start_sequence_a`: 코코볼
  - `start_sequence_b`: 그래놀라
- 두 번째 필드: 양
  - `small`: 적게
  - `medium`: 보통
  - `large`: 많이
- 세 번째 필드: 컵
  - `personal`: 개인컵
  - `store`: 매장컵

### OrderData 변환

`VoiceOrderPage`의 `_convertToOrderData()` 메서드에서 한글 주문 정보를 `OrderData` 형식으로 변환:

```dart
OrderData _convertToOrderData(String menu, String size, String cup) {
  // 메뉴: 코코볼/그래놀라 → start_sequence_a/start_sequence_b
  // 양: 많이/보통/적게 (그대로 사용)
  // 컵: 개인컵/매장컵 (그대로 사용)
}
```

## 🔍 디버깅

### 로그 확인

주요 로그 메시지:

- `[VoiceOrderPage] 주문 정보 수신: 메뉴=..., 양=..., 컵=...`
- `[VoiceOrderPage] 테스트 주문 CSV: ...`
- `[VoiceOrderPage] ROS2 토픽에 주문 정보 발행 시도...`
- `[VoiceOrderPage] ✅ 주문 정보 발행 성공. orderInfoStream 리스너가 처리합니다.`
- `[VoiceOrderPage] ❌ 주문 정보 발행 실패. /loading으로 직접 이동`
- `[VoiceOrderLoadingPage] 주문 완료! OrderCompletePage로 이동`

### 문제 해결

1. **테스트 버튼이 보이지 않는 경우**
   - `AppConfig.debugMode`가 `true`인지 확인

2. **토픽 발행이 실패하는 경우**
   - ROS2 연결 상태 확인 (`_statusService.isConnected`)
   - rosbridge 서버가 실행 중인지 확인

3. **주문 정보가 수신되지 않는 경우**
   - `/dsr01/kiosk/order` 토픽 구독 상태 확인
   - `orderInfoStream` 리스너가 정상적으로 등록되었는지 확인

## 📝 참고 사항

- `VoiceOrderLoadingPage`는 `LoadingPage`를 재사용하므로, `LoadingPage`의 변경사항이 자동으로 반영됩니다.
- 테스트 버튼은 개발 및 테스트 목적으로만 사용하며, 프로덕션에서는 `debugMode = false`로 설정해야 합니다.
- ROS2 토픽 발행 실패 시에도 주문 처리가 가능하도록 `/loading`으로 직접 이동하는 폴백 로직이 구현되어 있습니다.

## 🔗 관련 파일

- `cereal_order_app/lib/pages/voice_order_page.dart`
- `cereal_order_app/lib/pages/voice_order_loading_page.dart`
- `cereal_order_app/lib/pages/loading_page.dart`
- `cereal_order_app/lib/widgets/voice_order_test_button.dart`
- `cereal_order_app/lib/main.dart`
- `cereal_order_app/lib/config/app_config.dart`
- `cereal_order_app/lib/services/ros2_status_service.dart`

