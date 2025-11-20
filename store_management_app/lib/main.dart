import 'package:flutter/material.dart';
import 'package:store_management_app/pages/home_page.dart';
import 'package:store_management_app/constants/app_colors.dart';
import 'package:store_management_app/constants/app_strings.dart';

void main() {
  runApp(const StoreManagementApp());
}

class StoreManagementApp extends StatelessWidget {
  const StoreManagementApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: AppStrings.appTitle,
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(seedColor: AppColors.primary),
        useMaterial3: true,
        fontFamily: 'Pretendard',
      ),
      home: const HomePage(),
    );
  }
}
