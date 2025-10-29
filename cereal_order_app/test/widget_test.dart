// This is a basic Flutter widget test.
//
// To perform an interaction with a widget in your test, use the WidgetTester
// utility in the flutter_test package. For example, you can send tap and scroll
// gestures. You can also use WidgetTester to find child widgets in the widget
// tree, read text, and verify that the values of widget properties are correct.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

import 'package:cereal_order_app/main.dart';

void main() {
  testWidgets('App starts with WelcomePage', (WidgetTester tester) async {
    // 앱을 빌드하고 프레임 실행
    await tester.pumpWidget(const CerealOrderApp());

    // WelcomePage의 "시작하기" 버튼이 있는지 확인
    expect(find.text('시작하기'), findsOneWidget);
  });
}