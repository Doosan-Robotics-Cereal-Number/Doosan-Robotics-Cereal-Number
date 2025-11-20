import 'package:flutter/material.dart';
import '../services/ros2_service.dart';
import '../widgets/primary_action_button.dart';
import 'cleaning_request_done_page.dart';

class CleaningDetailPage extends StatefulWidget {
  final ROS2Service ros2Service;

  const CleaningDetailPage({
    super.key,
    required this.ros2Service,
  });

  @override
  State<CleaningDetailPage> createState() => _CleaningDetailPageState();
}

class _CleaningDetailPageState extends State<CleaningDetailPage> {
  final TextEditingController _textController = TextEditingController();
  final FocusNode _focusNode = FocusNode();
  bool _isFocused = false;
  bool _hasText = false;

  @override
  void initState() {
    super.initState();
    _focusNode.addListener(_handleFocusChange);
    _textController.addListener(_handleTextChange);
  }

  void _handleFocusChange() {
    if (_isFocused != _focusNode.hasFocus) {
      setState(() {
        _isFocused = _focusNode.hasFocus;
      });
    }
  }

  void _handleTextChange() {
    final hasText = _textController.text.trim().isNotEmpty;
    if (_hasText != hasText) {
      setState(() {
        _hasText = hasText;
      });
    }
  }

  @override
  void dispose() {
    _focusNode.removeListener(_handleFocusChange);
    _textController.removeListener(_handleTextChange);
    _focusNode.dispose();
    _textController.dispose();
    super.dispose();
  }

  void _handleRequest() {
    if (!_hasText) return;
    final text = _textController.text.trim();
    // TODO: 요청 처리 로직 추가 (text 사용)
    print('요청 텍스트: $text');

    Navigator.pushReplacement(
      context,
      MaterialPageRoute(
        builder: (context) => const CleaningRequestDonePage(),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color(0xFF121212),
      body: SafeArea(
        child: Column(
          children: [
            // 상단 바 (뒤로가기 버튼)
            Padding(
              padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 16),
              child: Row(
                children: [
                  GestureDetector(
                    onTap: () => Navigator.pop(context),
                    child: const Icon(
                      Icons.arrow_back,
                      color: Colors.white,
                      size: 24,
                    ),
                  ),
                ],
              ),
            ),

            // 메인 컨텐츠
            Expanded(
              child: SingleChildScrollView(
                padding: const EdgeInsets.symmetric(horizontal: 20),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.stretch,
                  children: [
                    const SizedBox(height: 40),
                    
                    // 알림 텍스트
                    const Text(
                      '앗! 손님이 두고간 컵을 발견했어요.',
                      textAlign: TextAlign.center,
                      style: TextStyle(
                        color: Colors.white,
                        fontSize: 24,
                        fontFamily: 'Pretendard',
                        fontWeight: FontWeight.w700,
                        height: 1.33,
                      ),
                    ),

                    const SizedBox(height: 40),

                    // 이미지 (모서리 둥글게 12px)
                    ClipRRect(
                      borderRadius: BorderRadius.circular(12),
                      child: Image.asset(
                        'assets/images/cup-1.png',
                        width: double.infinity,
                        height: 300,
                        fit: BoxFit.cover,
                        errorBuilder: (context, error, stackTrace) {
                          return Container(
                            width: double.infinity,
                            height: 300,
                            color: Colors.grey[800],
                            child: const Center(
                              child: Icon(
                                Icons.image,
                                color: Colors.grey,
                                size: 64,
                              ),
                            ),
                          );
                        },
                      ),
                    ),

                    const SizedBox(height: 40),

                    // 텍스트 입력창 (아웃라인 없이 텍스트만 표시)
                    TextField(
                      controller: _textController,
                      focusNode: _focusNode,
                      textAlign: TextAlign.center,
                      style: const TextStyle(
                        color: Colors.white,
                        fontSize: 24,
                        fontFamily: 'Pretendard',
                        fontWeight: FontWeight.w600,
                      ),
                      cursorColor: Colors.white,
                      decoration: InputDecoration(
                        hintText: _isFocused ? null : '컵 치워줘를 입력해보세요.',
                        hintStyle: TextStyle(
                          color: Colors.grey[500],
                          fontSize: 24,
                          fontFamily: 'Pretendard',
                          fontWeight: FontWeight.w600,
                        ),
                        border: InputBorder.none,
                        enabledBorder: InputBorder.none,
                        focusedBorder: InputBorder.none,
                        contentPadding: const EdgeInsets.symmetric(
                          vertical: 12,
                        ),
                      ),
                    ),

                    const SizedBox(height: 40),
                  ],
                ),
              ),
            ),

            // 요청하기 버튼
            PrimaryActionButton(
              label: '요청하기',
              onPressed: _hasText ? _handleRequest : null,
              enabled: _hasText,
              width: double.infinity,
              margin: const EdgeInsets.all(20),
            ),
          ],
        ),
      ),
    );
  }
}

