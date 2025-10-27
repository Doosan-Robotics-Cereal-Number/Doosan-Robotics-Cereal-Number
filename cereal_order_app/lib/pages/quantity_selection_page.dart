import 'package:flutter/material.dart';
import '../models/order_data.dart';
import '../widgets/selection_page_layout.dart';
import '../widgets/option_button.dart';

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
      contentArea: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          // 많이 옵션
          OptionButton(
            text: '많이',
            isSelected: selectedQuantity == '많이',
            onPressed: () {
              setState(() {
                selectedQuantity = '많이';
                orderData?.selectedQuantity = '많이';
              });
            },
          ),
          const SizedBox(height: 16),
          // 적당히 옵션
          OptionButton(
            text: '적당히',
            isSelected: selectedQuantity == '적당히',
            onPressed: () {
              setState(() {
                selectedQuantity = '적당히';
                orderData?.selectedQuantity = '적당히';
              });
            },
          ),
          const SizedBox(height: 16),
          // 적게 옵션
          OptionButton(
            text: '적게',
            isSelected: selectedQuantity == '적게',
            onPressed: () {
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
}


