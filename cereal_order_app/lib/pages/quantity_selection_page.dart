import 'package:flutter/material.dart';
import '../models/order_data.dart';
import '../widgets/selection_page_layout.dart';

class QuantitySelectionPage extends StatefulWidget {
  const QuantitySelectionPage({super.key});

  @override
  State<QuantitySelectionPage> createState() => _QuantitySelectionPageState();
}

class _QuantitySelectionPageState extends State<QuantitySelectionPage> {
  String? selectedQuantity = '적당히'; // 기본값으로 적당히 선택
  OrderData? orderData;

  @override
  void initState() {
    super.initState();
    // 다음 프레임에서 orderData 설정
    WidgetsBinding.instance.addPostFrameCallback((_) {
      setState(() {
        orderData?.selectedQuantity = '적당히';
      });
    });
  }

  @override
  Widget build(BuildContext context) {
    // 이전 화면에서 전달받은 주문 데이터 가져오기
    orderData = ModalRoute.of(context)?.settings.arguments as OrderData?;
    
    // 첫 빌드에서 orderData가 있으면 기본값 설정
    if (orderData != null && orderData!.selectedQuantity == null) {
      orderData!.selectedQuantity = '적당히';
    }
    
    return SelectionPageLayout(
      title: '양을 조절할 수 있어요!',
      backButtonText: '뒤로가기',
      confirmButtonText: '양 선택하기',
      isConfirmEnabled: selectedQuantity != null,
      showAppBar: false,
      onConfirmPressed: () {
        Navigator.pushNamed(
          context,
          '/loading',
          arguments: orderData,
        );
      },
      contentArea: Row(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          // 많이 카드
          _buildQuantityCard(
            label: '많이',
            grams: '(45g)',
            imagePath: 'assets/images/many-cereal.png',
            isSelected: selectedQuantity == '많이',
            onTap: () {
              setState(() {
                selectedQuantity = '많이';
                orderData?.selectedQuantity = '많이';
              });
            },
          ),
          const SizedBox(width: 60),
          // 적당히 카드
          _buildQuantityCard(
            label: '적당히',
            grams: '(30g)',
            imagePath: 'assets/images/normal-cereal.png',
            isSelected: selectedQuantity == '적당히',
            onTap: () {
              setState(() {
                selectedQuantity = '적당히';
                orderData?.selectedQuantity = '적당히';
              });
            },
          ),
          const SizedBox(width: 60),
          // 적게 카드
          _buildQuantityCard(
            label: '적게',
            grams: '(15g)',
            imagePath: 'assets/images/small-cereal.png',
            isSelected: selectedQuantity == '적게',
            onTap: () {
              setState(() {
                selectedQuantity = '적게';
                orderData?.selectedQuantity = '적게';
              });
            },
          ),
        ],
      ),
    );
  }

  Widget _buildQuantityCard({
    required String label,
    required String grams,
    required String imagePath,
    required bool isSelected,
    required VoidCallback onTap,
  }) {
    return GestureDetector(
      onTap: onTap,
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          // 이미지
          Container(
            decoration: BoxDecoration(
              color: Colors.white,
              borderRadius: BorderRadius.circular(20),
              border: Border.all(
                color: isSelected
                    ? const Color(0xFF0064FF)
                    : const Color(0xFFF1F1F1),
                width: 3,
              ),
            ),
            child: ClipRRect(
              borderRadius: BorderRadius.circular(17),
              child: Image.asset(
                imagePath,
                width: 246,
                height: 246,
                fit: BoxFit.cover,
                errorBuilder: (context, error, stackTrace) {
                  return const Icon(Icons.image_not_supported, size: 100);
                },
              ),
            ),
          ),
          const SizedBox(height: 30),
          // 라벨
          Text(
            label,
            style: const TextStyle(
              fontSize: 22,
              fontWeight: FontWeight.w600,
              color: Colors.black,
            ),
          ),
          const SizedBox(height: 8),
          // 그램 수
          Text(
            grams,
            style: const TextStyle(
              fontSize: 16,
              color: Color(0xFF666666),
            ),
          ),
        ],
      ),
    );
  }
}


