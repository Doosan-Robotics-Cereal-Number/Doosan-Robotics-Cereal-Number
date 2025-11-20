import 'package:flutter/material.dart';

class PrimaryActionButton extends StatelessWidget {
  const PrimaryActionButton({
    super.key,
    required this.label,
    required this.onPressed,
    this.enabled = true,
    this.width,
    this.height = 64,
    this.borderRadius = 12,
    this.margin,
  });

  final String label;
  final VoidCallback? onPressed;
  final bool enabled;
  final double? width;
  final double height;
  final double borderRadius;
  final EdgeInsetsGeometry? margin;

  @override
  Widget build(BuildContext context) {
    final bool isEnabled = enabled && onPressed != null;
    final button = GestureDetector(
      onTap: isEnabled ? onPressed : null,
      child: Container(
        width: width ?? double.infinity,
        height: height,
        decoration: ShapeDecoration(
          color: isEnabled ? const Color(0xFF237AFF) : Colors.grey[800],
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(borderRadius),
          ),
        ),
        child: Center(
          child: Text(
            label,
            textAlign: TextAlign.center,
            style: TextStyle(
              color: isEnabled ? Colors.white : Colors.grey[500],
              fontSize: 24,
              fontFamily: 'Pretendard',
              fontWeight: FontWeight.w700,
              height: 1.33,
            ),
          ),
        ),
      ),
    );

    if (margin != null) {
      return Padding(
        padding: margin!,
        child: button,
      );
    }
    return button;
  }
}

