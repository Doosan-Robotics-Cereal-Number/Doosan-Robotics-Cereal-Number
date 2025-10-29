import 'dart:convert';
import 'package:http/http.dart' as http;

/// Monitoring App으로 이슈 알림을 전송하는 서비스
class MonitoringNotifier {
  final String monitoringServerUrl;
  
  MonitoringNotifier({
    this.monitoringServerUrl = 'http://localhost:8080',
  });
  
  /// 이슈 알림 전송
  Future<bool> notifyIssue({
    required int issueFlag,
    String? orderInfo,
  }) async {
    try {
      final url = Uri.parse('$monitoringServerUrl/issue');
      final response = await http.post(
        url,
        headers: {
          'Content-Type': 'application/json',
        },
        body: jsonEncode({
          'timestamp': DateTime.now().toIso8601String(),
          'issueFlag': issueFlag,
          'orderInfo': orderInfo,
        }),
      ).timeout(
        const Duration(seconds: 2),
        onTimeout: () {
          print('⏱️ Monitoring 서버 응답 시간 초과');
          return http.Response('Timeout', 408);
        },
      );
      
      if (response.statusCode == 200) {
        print('✅ Monitoring 서버로 이슈 알림 전송 성공: issueFlag=$issueFlag');
        return true;
      } else {
        print('⚠️ Monitoring 서버 응답 오류: ${response.statusCode}');
        return false;
      }
    } catch (e) {
      print('❌ Monitoring 서버 알림 전송 실패: $e');
      return false;
    }
  }
}







