import re

def parse_order(text):
    """
    음성 인식된 텍스트에서 주문 정보 추출

    메뉴: 코코볼, 그래놀라
    양: 적게, 보통, 많이
    컵: 개인컵, 매장컵
    """

    order = {
        "menu": None,
        "size": None,
        "cup": None
    }

    # 텍스트 정규화 (공백 제거, 소문자 변환)
    text = text.replace(" ", "")
    text = text.lower()

    # 메뉴 찾기
    if "코코볼" in text or "코코불" in text or "코코" in text:
        order["menu"] = "코코볼"
    elif "그래놀라" in text or "그레놀라" in text or "그래" in text:
        order["menu"] = "그래놀라"

    # 양 찾기
    if "적게" in text or "작게" in text or "조금" in text:
        order["size"] = "적게"
    elif "보통" in text or "중간" in text or "미디엄" in text:
        order["size"] = "보통"
    elif "많이" in text or "크게" in text or "라지" in text:
        order["size"] = "많이"

    # 컵 찾기
    if "개인" in text or "개인컵" in text or "텀블러" in text:
        order["cup"] = "개인컵"
    elif "매장" in text or "매장컵" in text or "일회용" in text:
        order["cup"] = "매장컵"

    return order

def validate_order(order):
    """주문이 완전한지 확인"""
    return all([order["menu"], order["size"], order["cup"]])

def format_order_text(order):
    """주문을 자연스러운 문장으로 변환"""
    if not validate_order(order):
        missing = []
        if not order["menu"]:
            missing.append("메뉴")
        if not order["size"]:
            missing.append("양")
        if not order["cup"]:
            missing.append("컵 종류")

        return f"{', '.join(missing)}를 말씀해주세요."

    return f"{order['menu']} {order['size']} 사이즈 {order['cup']}으로 주문하시겠습니까?"

# 테스트
if __name__ == "__main__":
    test_cases = [
        "코코볼 보통으로 매장 컵 주세요",
        "그래놀라 많이 개인 컵이요",
        "코코볼 적게요",
        "보통 사이즈로 매장 컵",
        "코코 많이 텀블러",
    ]

    print("="*60)
    print("주문 파싱 테스트")
    print("="*60)

    for text in test_cases:
        print(f"\n입력: '{text}'")
        order = parse_order(text)
        print(f"파싱 결과: {order}")
        print(f"완전한 주문: {validate_order(order)}")
        print(f"응답: {format_order_text(order)}")
